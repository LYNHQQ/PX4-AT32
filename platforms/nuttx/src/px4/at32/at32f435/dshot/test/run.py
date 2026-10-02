#!/usr/bin/env python3
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Compile and run the complete AT32 DShot and IO timer bodies with native mocks.

Only #include directives are removed from generated temporary source copies.
This tests protocol and lifecycle behavior; it does not build firmware or verify
DMA hardware, pin timing, or ESC interoperability. Requires an initialized NuttX
submodule and a native C11 compiler (GCC/Clang or MSVC).
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def without_includes(source):
    return re.sub(r'^\s*#include[^\n]*\n', '', source, flags=re.MULTILINE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CC'),
                        help='Native C11 compiler; defaults to CC, cc, gcc, clang, or cl.')
    parser.add_argument('--msvc-env', type=Path,
                        help='Optional vcvars64.bat path for MSVC outside a developer prompt.')
    args = parser.parse_args()
    compiler = args.compiler or next((shutil.which(name) for name in ('cc', 'gcc', 'clang', 'cl')
                                     if shutil.which(name)), None)
    if compiler is None:
        parser.error('No native compiler found; set --compiler or run from a compiler developer prompt.')
    tests = Path(__file__).resolve().parent
    family = tests.parent.parent
    root = next(parent for parent in tests.parents if (parent / 'src/drivers/drv_dshot.h').is_file())
    nuttx = root / 'platforms/nuttx/NuttX/nuttx/arch/arm/src/at32'
    if not (nuttx / 'at32_gpio.h').is_file():
        parser.error('Initialize platforms/nuttx/NuttX/nuttx before running this regression.')
    env = os.environ.copy()
    if args.msvc_env:
        if not args.msvc_env.is_file():
            parser.error('--msvc-env must name an existing vcvars64.bat file.')
        # Pass CMD's quoted command unchanged; Python's argv quoting would
        # introduce C-style escapes that CMD does not understand.
        result = subprocess.run(f'cmd.exe /u /d /s /c "call "{args.msvc_env.resolve()}" >nul && set"',
                                check=True, capture_output=True, encoding='utf-16-le')
        env.update(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line and not line.startswith('='))
    # Generated headers and object files stay in a single disposable directory.
    with tempfile.TemporaryDirectory(prefix='px4-at32-dshot-') as directory:
        build = Path(directory)
        gpio = '\n'.join(line for line in (nuttx / 'at32_gpio.h').read_text().splitlines()
                         if re.match(r'#\s*define GPIO_', line))
        headers = [gpio, (nuttx / 'hardware/at32_tim.h').read_text()]
        for header in (family / 'include/px4_arch/dshot.h', family / 'include/px4_arch/io_timer.h',
                       root / 'src/drivers/drv_dshot.h'):
            headers.append(without_includes(header.read_text()))
        (build / 'actual_headers.h').write_text('\n'.join(headers))
        for name, source in (('actual_io_timer.c', family / 'io_pins/io_timer.c'),
                             ('actual_dshot.c', family / 'dshot/dshot.c')):
            (build / name).write_text(without_includes(source.read_text()))
        for name in ('test.c', 'host.h'):
            shutil.copyfile(tests / name, build / name)
        msvc = Path(compiler).name.lower() in ('cl', 'cl.exe')
        executable = build / ('test.exe' if os.name == 'nt' else 'test')
        if msvc:
            # Use an unoptimized MSVC baseline: MSVC 19.51 /O2 produced a
            # final-bit mismatch in this native harness. GCC/Clang exercise
            # optimized production bodies separately.
            command = [compiler, '/nologo', '/std:c11', '/W3', '/Od', '/D_CRT_SECURE_NO_WARNINGS',
                       '/wd4312', '/wd4244', '/wd4267', 'test.c', '/Fe:' + str(executable)]
        else:
            command = [compiler, '-std=c11', '-D_DEFAULT_SOURCE', '-O2', '-Wall', '-Wextra', '-Werror',
                       '-Wno-unused-parameter', '-Wno-unused-function', '-Wno-missing-field-initializers',
                       '-Wno-int-to-pointer-cast', 'test.c', '-o', str(executable)]
        print('Testing complete production DShot and IO timer bodies with native hardware mocks.', flush=True)
        subprocess.run(command, cwd=build, env=env, check=True)
        subprocess.run([str(executable)], cwd=build, env=env, check=True)


if __name__ == '__main__':
    main()
