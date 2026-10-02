#!/usr/bin/env python3
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Check native AT32 USB GCCFG writes with register mocks.

Requires an initialized NuttX submodule and a native C compiler. Run from any
directory; --compiler can name a compiler outside PATH. The test extracts the
production GCCFG initialization and includes its actual hardware header. It
checks the register layout for all VBUS/SOF configurations, not USB enumeration
or physical clock and signal timing.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='cc', help='Native GCC or Clang executable.')
    args = parser.parse_args()
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / 'src/drivers/drv_dshot.h').is_file())
    chip = root / 'platforms/nuttx/NuttX/nuttx/arch/arm/src/at32'
    source_file = chip / 'at32_otgfsdev.c'
    if not source_file.is_file():
        parser.error('Initialize platforms/nuttx/NuttX/nuttx before running this regression.')
    source = source_file.read_text()
    function = source.index('static void at32_hwinitialize(struct at32_usbdev_s *priv)\n{')
    start = source.index('  /* Deactivate the power down */', function)
    end = source.index('  up_mdelay(20);', start)
    actual = source[start:end]
    hardware = chip / 'hardware'
    prefix = '#include <stdint.h>\n'
    for name in ('at32f43xxx_memorymap.h', 'at32fxxxxx_otgfs.h'):
        prefix += '#include "' + (hardware / name).as_posix() + '"\n'
    prefix += r'''
static uint32_t written, address, writes;
static void at32_putreg(uint32_t value, uint32_t reg)
{
  written = value;
  address = reg;
  writes++;
}
static void configure(void)
{
  uint32_t regval;
'''
    checks = r'''
}
int main(void)
{
  configure();
  if (writes != 1 || address != 0x50000038u)
    return 1;
  /* AT32 SDK: bits 18 and 19 are reserved, not STM32 VBUS sensing enables. */
  if ((written & 0x000c0000u) != 0)
    return 2;
  if (written != EXPECTED_VALUE)
    return 3;
  return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix='at32-usb-test-') as temp:
        directory = Path(temp)
        test = directory / 'gccfg.c'
        test.write_text(prefix + actual + checks)
        for vbus in (False, True):
            for sof in (False, True):
                expected = 0x00010000 | (0 if vbus else 0x00200000) | (0x00100000 if sof else 0)
                flags = [f'-DEXPECTED_VALUE=0x{expected:08x}u']
                if vbus:
                    flags.append('-DCONFIG_USBDEV_VBUSSENSING=1')
                if sof:
                    flags.append('-DCONFIG_AT32_OTGFS_SOFOUTPUT=1')
                executable = directory / f'vbus{int(vbus)}-sof{int(sof)}.exe'
                subprocess.run([args.compiler, '-O2', '-std=c11', '-Wall', '-Wextra',
                                '-Werror', *flags, str(test), '-o', str(executable)], check=True)
                subprocess.run([str(executable)], check=True)
                print(f'PASS native GCCFG VBUS={int(vbus)} SOF={int(sof)} value=0x{expected:08x}')


if __name__ == '__main__':
    main()
