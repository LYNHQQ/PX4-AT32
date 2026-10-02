"""Check the build_all_targets matrix that generate_board_targets_json.py emits.

The generator decides what CI builds, and its mistakes are silent: a board
dropped from every group is never built, two groups with the same name
overwrite each other's artifacts and caches, and a group whose cache
namespace has no seeder always starts cold. These tests run the generator
the way the workflow does and check its output against the board tree.
"""

import json
import re
import subprocess
import sys
import unittest
from collections import Counter
from pathlib import Path

from build_all_runner import METADATA_TARGETS, build_dir_name

ROOT = Path(__file__).resolve().parents[2]
GENERATOR = ROOT / 'Tools' / 'ci' / 'generate_board_targets_json.py'
BOARDS = ROOT / 'boards'

# Workflow expressions read these keys from every build matrix entry.
MATRIX_KEYS = {'container', 'runner', 'group', 'targets', 'slots',
               'cache_prefix', 'cache_size'}


def generate(*args):
    # the generator resolves boards/ relative to the repo root, as in CI
    output = subprocess.run([sys.executable, str(GENERATOR), *args], cwd=ROOT,
                            check=True, capture_output=True, text=True).stdout
    return json.loads(output)['include']


def companion_targets():
    """Targets a parent target builds itself (boards/*/*/companion_targets)."""
    companions = set()
    for path in BOARDS.glob('*/*/companion_targets'):
        companions |= {line.strip() for line in path.read_text().splitlines()
                       if line.strip() and not line.startswith('#')}
    return companions


class BuildMatrixTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flat = [entry['target'] for entry in generate()]
        cls.groups = generate('--group')
        cls.seeders = generate('--group', '--seeders')
        cls.grouped = [t for g in cls.groups for t in g['targets']]

    def test_every_target_built_exactly_once(self):
        duplicates = [t for t, n in Counter(self.grouped).items() if n > 1]
        self.assertEqual(duplicates, [])
        companions = companion_targets()
        missing = set(self.flat) - companions - set(self.grouped)
        self.assertEqual(missing, set(), 'targets no build job compiles')
        # a companion is already built by its parent target's make rule
        self.assertEqual(companions & set(self.grouped), set())

    def test_runner_knows_every_non_board_target(self):
        # Targets that are not board configs build inside another target's
        # build/ directory; the runner must map them, or two slots write the
        # same directory and one overwrites the other.
        extra = set(self.grouped) - set(self.flat)
        self.assertLessEqual(extra, METADATA_TARGETS)
        for target in extra:
            self.assertEqual(build_dir_name(target), 'px4_sitl_default')

    def test_group_names_unique(self):
        # group names key the uploaded artifact and the ccache namespace
        names = Counter(g['group'] for g in self.groups)
        self.assertEqual([n for n, count in names.items() if count > 1], [])

    def test_matrix_entries_match_workflow(self):
        for g in self.groups:
            with self.subTest(group=g.get('group')):
                self.assertLessEqual(MATRIX_KEYS, set(g))
                self.assertIsInstance(g['targets'], list)
                self.assertTrue(g['targets'])
                self.assertEqual(g['len'], len(g['targets']))
                self.assertIsInstance(g['slots'], int)
                self.assertGreaterEqual(g['slots'], 1)

    def test_every_cache_namespace_has_a_seeder(self):
        # the generator seeds special groups from nothing on purpose; every
        # other group must restore from a seeder in its own namespace
        seeded = {s['cache_prefix'] for s in self.seeders}
        for g in self.groups:
            if g['chip_family'] == 'special':
                continue
            with self.subTest(group=g['group']):
                self.assertIn(g['cache_prefix'], seeded)

    def test_board_group_matches_its_nuttx_chip_family(self):
        # a chip family that the mapping in loadconfig.py does not know about
        # silently degrades to 'native', so the board joins another family's
        # group without any other test noticing
        family_of = {t: g['chip_family'] for g in self.groups for t in g['targets']}
        checked = 0
        for px4board in BOARDS.glob('*/*/*.px4board'):
            manufacturer, board = px4board.parts[-3], px4board.parts[-2]
            label = px4board.name[:-len('.px4board')]
            target = '_'.join((manufacturer, board, label))
            family = family_of.get(target)
            if family is None or family == 'special':
                continue  # not built by CI, or isolated by label
            expected = reference_chip_family(px4board.parent)
            if expected is None:
                continue  # not a NuttX board, so no chip family to map
            with self.subTest(target=target):
                self.assertEqual(family, expected)
            checked += 1
        self.assertGreater(checked, 100)


def reference_chip_family(board_dir):
    """Chip family of a board, reimplemented from loadconfig.py's contract.

    Returns None for boards without a NuttX arch chip (posix/qurt targets).
    """
    text = None
    for name in ('nsh', 'bootloader'):
        defconfig = board_dir / 'nuttx-config' / name / 'defconfig'
        if defconfig.is_file():
            text = defconfig.read_text()
            break
    if text is None:
        return None
    match = re.search(r'^CONFIG_ARCH_CHIP="([^"]+)"', text, re.MULTILINE)
    if match is None:
        return None
    arch_chip = match.group(1)
    if arch_chip.startswith('s32k'):
        return 's32k'
    if arch_chip == 'stm32':
        return 'stm32f1' if re.search(r'^CONFIG_ARCH_CHIP_STM32F1', text, re.MULTILINE) else 'stm32f4'
    return arch_chip


if __name__ == '__main__':
    unittest.main()
