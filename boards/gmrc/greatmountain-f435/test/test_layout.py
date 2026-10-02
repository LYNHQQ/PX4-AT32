#!/usr/bin/env python3
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Check stack alignment and zero-wait code placement in actual build artifacts.

Run from any directory with --bin, --elf and --map pointing to one board build.
--nm can name the ARM binutils executable outside PATH.
The native vector and heap base use _ebss + CONFIG_IDLETHREAD_STACKSIZE,
so both BSS end and the configured stack size must preserve 8-byte alignment.
Estimator/IMU/rate control and common wakeup/memory/serial text must fit the
EOPB0=2 256 KiB zero-wait window.
This checks image/layout properties, not stack capacity or execution timing.
"""

import argparse
import base64
from fnmatch import fnmatchcase
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import zlib


ZERO_WAIT_END = 0x08040000
PARAMETER_BASE = 0x083ee000
RAM_FUNCTION_LIMIT = 24 * 1024


def memory_region(linker, name):
    memory = re.search(r'MEMORY\s*\{(.*?)\}', linker, re.DOTALL)
    if memory is None:
        raise ValueError('Linker script has no MEMORY block')
    match = re.search(name + r'\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+)'
                      r'\s*,\s*LENGTH\s*=\s*(\d+)([KM]?)\b', memory[1])
    if match is None:
        raise ValueError(f'Linker MEMORY has no {name} origin/length')
    return int(match[1], 16), int(match[2]) * {'': 1, 'K': 1024, 'M': 1024 * 1024}[match[3]]


def elf_layout(path):
    raw = path.read_bytes()
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', raw)
    if header[0][:6] != b'\x7fELF\x01\x01' or header[1:3] != (2, 40):
        raise ValueError('Expected an executable little-endian ARM ELF32')
    loads = [struct.unpack_from('<IIIIIIII', raw, header[5] + i * header[9])
             for i in range(header[10])]
    loads = [load for load in loads if load[0] == 1]
    entries = [struct.unpack_from('<IIIIIIIIII', raw, header[6] + i * header[11])
               for i in range(header[12])]
    strings_header = entries[header[13]]
    strings = raw[strings_header[4]:strings_header[4] + strings_header[5]]
    sections = {}
    for entry in entries:
        name = strings[entry[0]:strings.index(b'\0', entry[0])].decode()
        sections[name] = {'type': entry[1], 'flags': entry[2], 'address': entry[3],
                          'offset': entry[4], 'size': entry[5]}
    return raw, header[4], loads, sections


def load_address(section, loads):
    matches = [load for load in loads
               if load[2] <= section['address'] and
               section['address'] + section['size'] <= load[2] + load[4] and
               section['offset'] == load[1] + section['address'] - load[2]]
    if len(matches) != 1:
        raise ValueError('ELF section has no unique Flash load mapping')
    return matches[0][3] + section['address'] - matches[0][2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin', type=Path, required=True, help='Built board firmware BIN.')
    parser.add_argument('--elf', type=Path, required=True, help='Matching board firmware ELF.')
    parser.add_argument('--map', type=Path, required=True, help='Matching GNU linker map.')
    parser.add_argument('--px4', type=Path, help='Defaults to the BIN path with .px4 suffix.')
    parser.add_argument('--nm', default='arm-none-eabi-nm', help='ARM binutils nm executable.')
    parser.add_argument('--objcopy', help='Defaults to nm with -nm replaced by -objcopy.')
    args = parser.parse_args()
    board = Path(__file__).resolve().parents[1]
    config = (board / 'nuttx-config/nsh/defconfig').read_text(encoding='utf-8')
    linker = (board / 'nuttx-config/scripts/script.ld').read_text(encoding='utf-8')
    prototype = json.loads((board / 'firmware.prototype').read_text(encoding='utf-8'))
    origin, flash_size = memory_region(linker, 'flash')
    sram, sram_size = memory_region(linker, 'sram')
    if origin != 0x08010000 or origin + flash_size != PARAMETER_BASE:
        parser.exit(1, 'FAIL linker must reserve 64 KiB for bootloader and 8 KiB for parameters\n')
    if (sram, sram_size) != (0x20000000, 384 * 1024):
        parser.exit(1, 'FAIL linker SRAM does not match AT32F435VM\n')
    for option in ('CONFIG_ARCH_RAMFUNCS=y', 'CONFIG_AT32_RAMFUNCS=y'):
        if option not in config.splitlines():
            parser.exit(1, f'FAIL SRAM execution requires {option}\n')
    if re.search(r'^CONFIG_GRAN(?:_[A-Z0-9_]+)?=y$', config, re.MULTILINE):
        parser.exit(1, 'FAIL unused granule DMA allocator must not be enabled\n')

    stack = re.search(r'^CONFIG_IDLETHREAD_STACKSIZE=(\d+)$', config, re.MULTILINE)
    if stack is None or int(stack[1]) <= 0 or int(stack[1]) % 8:
        parser.exit(1, 'FAIL idle stack size must be a positive multiple of 8 bytes\n')
    bss = re.search(r'\.bss\s*:\s*\{(.*?)\}\s*>\s*sram', linker, re.DOTALL)
    if bss is None or not re.search(r'\.\s*=\s*ALIGN\(8\);\s*_ebss\s*=\s*ABSOLUTE\(\.\);', bss[1]):
        parser.exit(1, 'FAIL BSS end must be aligned to 8 bytes before _ebss\n')

    image = args.bin.read_bytes()
    vector = image[:8]
    if len(vector) != 8:
        parser.exit(1, 'FAIL BIN is missing its initial stack and reset vectors\n')
    initial_sp, reset = struct.unpack('<II', vector)
    if initial_sp % 8 or not sram < initial_sp <= sram + sram_size:
        parser.exit(1, f'FAIL actual BIN initial MSP 0x{initial_sp:08x} is not 8-byte aligned\n')

    result = subprocess.run([args.nm, '-S', '-n', '-C', str(args.elf)], check=True,
                            capture_output=True, text=True)
    matches = [match for line in result.stdout.splitlines()
               if (match := re.fullmatch(r'([0-9a-fA-F]+)(?:\s+([0-9a-fA-F]+))?\s+(\S)\s+(.+)', line))]
    symbols = {match[4]: int(match[1], 16) for match in matches}
    sizes = {match[4]: int(match[2], 16) for match in matches if match[2]}
    if 'g_dma_heap' in symbols:
        parser.exit(1, 'FAIL unused DMA heap still consumes static SRAM\n')
    required = ('_vectors', 'start', '__start', 'g_idle_topstack', '_ebss', '_szw_hot', '_ezw_hot',
                '_szw_common', '_ezw_common', '_sramfuncs', '_eramfuncs', '_framfuncs',
                '_sdata', '_edata', '_sbss', '_eronly')
    if any(name not in symbols for name in required):
        parser.exit(1, 'FAIL ELF is missing vector, data, RAM-copy or hot-code boundaries\n')
    if symbols['_vectors'] != origin:
        parser.exit(1, f'FAIL ELF vectors 0x{symbols["_vectors"]:08x} disagree with current linker 0x{origin:08x}\n')
    if initial_sp != symbols['_ebss'] + int(stack[1]) or reset != (symbols['start'] | 1):
        parser.exit(1, 'FAIL BIN vector does not match the ELF stack/reset layout\n')

    mapping = args.map.read_text(encoding='utf-8')
    for name, expected in (('flash', (origin, flash_size)), ('sram', (sram, sram_size))):
        match = re.search(r'^' + name + r'\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\b', mapping, re.MULTILINE)
        if match is None or (int(match[1], 16), int(match[2], 16)) != expected:
            parser.exit(1, f'FAIL map {name} origin/length disagree with current linker\n')
    hot_start, hot_end = symbols['_szw_hot'], symbols['_ezw_hot']
    if not (origin < hot_start < hot_end <= ZERO_WAIT_END):
        parser.exit(1, 'FAIL hot text exceeds physical zero-wait Flash end 0x08040000\n')
    if not reset & 1 or not origin <= (reset & ~1) < ZERO_WAIT_END:
        parser.exit(1, 'FAIL reset vector is outside zero-wait application Flash\n')
    vector_size = sizes.get('_vectors', 0)
    if vector_size < 8 or vector_size % 4 or vector_size > hot_start - origin:
        parser.exit(1, 'FAIL ELF vector table has an invalid size\n')
    handlers = struct.unpack('<' + 'I' * (vector_size // 4), image[:vector_size])[1:]
    if any(handler and (not handler & 1 or not origin <= (handler & ~1) < ZERO_WAIT_END)
           for handler in handlers):
        parser.exit(1, 'FAIL vector handler is outside Thumb zero-wait application Flash\n')
    common_start, common_end = symbols['_szw_common'], symbols['_ezw_common']
    if not (hot_start < common_start < common_end == hot_end):
        parser.exit(1, 'FAIL common runtime block must lie within the zero-wait hot layout\n')
    for name in ('_szw_hot', '_ezw_hot', '_szw_common', '_ezw_common', '_sramfuncs',
                 '_eramfuncs', '_framfuncs', '_sdata', '_edata', '_sbss', '_ebss', '_eronly'):
        match = re.search(r'^\s*(0x[0-9a-fA-F]+)\s+' + name + r'\s*=', mapping, re.MULTILINE)
        if match is None or int(match[1], 16) != symbols[name]:
            parser.exit(1, f'FAIL linker map and ELF disagree on {name}\n')

    objcopy = args.objcopy or str(Path(args.nm).with_name(Path(args.nm).name.replace('-nm', '-objcopy')))
    with tempfile.TemporaryDirectory(prefix='px4-at32-layout-') as directory:
        extracted = Path(directory) / 'from-elf.bin'
        subprocess.run([objcopy, '-O', 'binary', str(args.elf), str(extracted)], check=True)
        if extracted.read_bytes() != image:
            parser.exit(1, 'FAIL complete BIN differs from ELF load image\n')
    package = json.loads((args.px4 or args.bin.with_suffix('.px4')).read_text(encoding='utf-8'))
    if (zlib.decompress(base64.b64decode(package['image'])) != image or
            package['image_size'] != len(image) or package['image_maxsize'] != flash_size or
            prototype['image_maxsize'] != flash_size or package['board_id'] != prototype['board_id'] or
            package['magic'] != 'PX4FWv1'):
        parser.exit(1, 'FAIL PX4 payload/metadata disagree with BIN and current partition\n')
    raw, entry, loads, sections = elf_layout(args.elf)
    if (entry & ~1) != (symbols['__start'] & ~1) or not entry & 1:
        parser.exit(1, 'FAIL ELF entry is not the native startup function\n')
    ram = sections['.ramfunc']
    start, end, load = (symbols[name] for name in ('_sramfuncs', '_eramfuncs', '_framfuncs'))
    if (ram['address'] != start or ram['address'] + ram['size'] != end or
            not sram <= start < end <= sram + sram_size or ram['size'] > RAM_FUNCTION_LIMIT or
            not ram['flags'] & 4 or any(value % 8 for value in (start, end, load)) or
            load_address(ram, loads) != load):
        parser.exit(1, 'FAIL executable SRAM output/copy range, alignment or 24 KiB capacity\n')
    if not end <= symbols['_sdata'] <= symbols['_edata'] <= symbols['_sbss'] <= symbols['_ebss']:
        parser.exit(1, 'FAIL RAM functions, data, BSS or idle stack overlap\n')
    ram_functions = ('up_dshot_trigger', 'output_complete', 'capture_complete',
                     'io_timer_set_enable', 'io_timer_set_dshot_capture_mode',
                     'uORB::DeviceMaster::getDeviceNodeLocked(',
                     'uORB::DeviceNode::publish(', 'uORB::Manager::orb_publish(',
                     'Mavlink::task_main(', '__aeabi_dadd', 'sqrt',
                     'ControlAllocator::Run(', 'ControlAllocator::publish_actuator_controls(',
                     'ControlAllocationSequentialDesaturation::')
    for prefix in ram_functions:
        matches = [(name, address) for name, address in symbols.items()
                   if (name.startswith(prefix) or name.startswith('non-virtual thunk to ' + prefix))
                   and (prefix != 'sqrt' or name == prefix)]
        if not matches or any(not start <= address < address + sizes[name] <= end
                              for name, address in matches):
            parser.exit(1, f'FAIL selected SRAM function {prefix} is missing or outside RAM copy range\n')

    # Startup/reconfiguration does not justify displacing a running control path.
    # Check actual symbols so a broad archive selector cannot silently undo this.
    cold_functions = ('Ekf::Ekf(*', 'Ekf::~Ekf(*', 'up_dshot_init', 'up_dshot_arm',
                      'up_bdshot_status', 'io_timer_*allocate_*', 'io_timer_*init*',
                      'io_timer_set_pwm_rate', 'up_pwm_servo_*init')
    for pattern in cold_functions:
        matches = [(name, address) for name, address in symbols.items()
                   if fnmatchcase(name, pattern)]
        if not matches or any(not hot_end <= address < address + sizes[name] <= PARAMETER_BASE
                              for name, address in matches):
            parser.exit(1, f'FAIL cold function {pattern} is missing or outside ordinary Flash\n')
    if load_address(sections['.data'], loads) != symbols['_eronly']:
        parser.exit(1, 'FAIL data Flash LMA disagrees with native startup source\n')
    max_load_end = origin
    for section in sections.values():
        if not section['flags'] & 2 or not section['size']:
            continue
        address, size = section['address'], section['size']
        if not ((origin <= address < address + size <= PARAMETER_BASE) or
                (sram <= address < address + size <= sram + sram_size)):
            parser.exit(1, 'FAIL allocated ELF section exceeds firmware/SRAM boundaries\n')
        if section['type'] != 8:
            lma = load_address(section, loads)
            if not origin <= lma < lma + size <= PARAMETER_BASE:
                parser.exit(1, 'FAIL ELF load section overlaps bootloader/parameter pages\n')
            if raw[section['offset']:section['offset'] + size] != image[lma - origin:lma - origin + size]:
                parser.exit(1, 'FAIL full ELF load section differs from BIN, including RAM veneers\n')
            max_load_end = max(max_load_end, lma + size)
    if max_load_end != origin + len(image):
        parser.exit(1, 'FAIL BIN extent disagrees with complete ELF Flash load extent\n')
    idle = symbols['g_idle_topstack']
    section = next(value for value in sections.values()
                   if value['type'] != 8 and value['address'] <= idle < value['address'] + value['size'])
    if struct.unpack_from('<I', raw, section['offset'] + idle - section['address'])[0] != initial_sp:
        parser.exit(1, 'FAIL native idle stack constant disagrees with vector MSP\n')

    # The current linker prioritizes the Ekf core, not the whole EKF2 module.
    core = ('Ekf::predictCovariance(', 'OutputPredictor::calculateOutputStates(',
            'sensors::VehicleIMU::Run(', 'sensors::VehicleAngularVelocity::Run(',
            'MulticopterRateControl::Run(', 'RateControl::update(',
            'sensors::VehicleAngularVelocity::FilterAngularVelocity(',
            'sensors::VehicleAngularVelocity::CalibrateAndPublish(',
            'PX4Gyroscope::updateFIFO(', 'ICM42688P::RunImpl(', 'spi_exchange',
            'AttitudeControl::', 'ImuDownSampler::', 'EstimatorInterface::setIMUData(')
    for prefix in core:
        matches = [(name, address) for name, address in symbols.items()
                   if name.startswith(prefix) and (prefix != 'spi_exchange' or name == prefix)]
        if not matches or any(not hot_start <= address < address + sizes[name] <= hot_end
                              for name, address in matches):
            parser.exit(1, f'FAIL core function {prefix} is missing or outside zero-wait hot text\n')

    common = ('memcpy', 'memset', 'nxsem_wait', 'nxsem_post', 'nxsig_clockwait',
              'nxsched_self', 'uart_writev', 'cdcuart_sendbuf', 'cdcuart_dmasend')
    for prefix in common:
        matches = [(name, address) for name, address in symbols.items() if name.startswith(prefix)]
        if not matches or any(not common_start <= address < common_end for _, address in matches):
            parser.exit(1, f'FAIL common function {prefix} is missing or outside its zero-wait block\n')

    print(f'PASS board layout and actual BIN initial MSP 0x{initial_sp:08x} are 8-byte aligned; '
          f'idle stack {stack[1]} bytes, reset vector 0x{reset:08x}')
    print(f'PASS ELF/map hot text [0x{hot_start:08x}, 0x{hot_end:08x}) '
          f'({hot_end - hot_start} bytes) ends before physical 0x{ZERO_WAIT_END:08x}; {len(core)} core functions')
    print(f'PASS common runtime text [0x{common_start:08x}, 0x{common_end:08x}) '
          f'({common_end - common_start} bytes); {len(common)} memory/wakeup/serial function families')
    print(f'PASS matching full BIN/ELF/PX4 load bytes; firmware {len(image)} / {flash_size} bytes, '
          f'RAM copy {ram["size"]} / {RAM_FUNCTION_LIMIT} bytes including veneers; '
          f'{len(ram_functions)} SRAM function families, {len(handlers)} vector handlers')
    print(f'PASS {len(cold_functions)} startup/reconfiguration families remain in ordinary Flash; '
          'unused DMA heap and granule allocator absent')


if __name__ == '__main__':
    main()
