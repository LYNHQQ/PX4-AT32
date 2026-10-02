#!/usr/bin/env python3
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Check the pinned NuttX AT32F435 erase function with native register mocks.

Requires an initialized NuttX submodule, Clang with the wasm32 target and
WebAssembly linker, and Node.js. Run this script from any working directory;
--compiler and --node can name executables outside PATH. Temporary artifacts
are removed after the test. This checks register addressing and return values,
not physical flash operations or firmware integration.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='clang', help='Clang executable with wasm32 support.')
    parser.add_argument('--node', default='node', help='Node.js executable.')
    args = parser.parse_args()
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / 'src/drivers/drv_dshot.h').is_file())
    source_file = root / 'platforms/nuttx/NuttX/nuttx/arch/arm/src/at32/at32f43xx_flash.c'
    if not source_file.is_file():
        parser.error('Initialize platforms/nuttx/NuttX/nuttx before running this regression.')
    source = source_file.read_text()
    start = source.index('ssize_t up_progmem_eraseblock(')
    end = source.index('\nssize_t up_progmem_write(', start)
    actual = source[start:end]
    prefix = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int32_t ssize_t;
// CONFIG_AT32_FLASH_CONFIG_M: 4032 KiB, uniform 4 KiB pages, two banks.
#define AT32_FLASH_BASE 0x08000000u
#define AT32_FLASH_PAGESIZE 4096u
#define AT32_FLASH_NPAGES 1008u
#define AT32_FLASH_BANK2_START 512u
// Mock slots: the address registers latch the written value and read back zero.
#define AT32_FLASH_CTRL 0u
#define AT32_FLASH_ADDR 1u
#define AT32_FLASH_STS 2u
#define AT32_FLASH_CTRL2 3u
#define AT32_FLASH_ADDR2 4u
#define AT32_FLASH_STS2 5u
#define FLASH_CTRL_SECERS (1u << 1)
#define FLASH_CTRL_ERSTR (1u << 6)
#define FLASH_STS_OBF (1u << 0)
#define EFAULT 14
#define EIO 5
static uint32_t regs[6], addr_latch[2], erased[AT32_FLASH_NPAGES], erasures, g_lock;
static uint32_t getreg32(unsigned addr) {
    // A write-only address register returns zero, not the last address written.
    return (addr == AT32_FLASH_ADDR || addr == AT32_FLASH_ADDR2) ? 0 : regs[addr];
}
static void putreg32(uint32_t value, unsigned addr) {
    if (addr == AT32_FLASH_ADDR) addr_latch[0] = value;
    else if (addr == AT32_FLASH_ADDR2) addr_latch[1] = value;
    regs[addr] = value;
}
static void modifyreg32(unsigned addr, uint32_t clear, uint32_t set) {
    putreg32((getreg32(addr) & ~clear) | set, addr);
    if ((addr == AT32_FLASH_CTRL || addr == AT32_FLASH_CTRL2)
        && (set & FLASH_CTRL_ERSTR)) {
        uint32_t target = addr_latch[addr == AT32_FLASH_CTRL ? 0 : 1];
        uint32_t page = (target - AT32_FLASH_BASE) / AT32_FLASH_PAGESIZE;
        if (page < AT32_FLASH_NPAGES) erased[page] = 1;
        erasures++;
        regs[addr] &= ~FLASH_CTRL_ERSTR;
    }
}
static void flash_unlock(void) { }
static void at32_waste(void) { }
static void nxmutex_lock(uint32_t *mutex) { (*mutex)++; }
static void nxmutex_unlock(uint32_t *mutex) { (*mutex)--; }
static uint32_t up_progmem_ispageerased(size_t page) { return erased[page] ? 0 : 1; }
static size_t up_progmem_pagesize(size_t page) { (void)page; return AT32_FLASH_PAGESIZE; }
'''
    tests = r'''
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int run_tests(void) {
    const unsigned sequences[][3] = {{1006,1007,1006}, {1007,1006,1007}, {0,1,0}, {1,0,1}};
    for (unsigned s=0; s<4; s++) {
        for (unsigned r=0; r<6; r++) regs[r]=0;
        for (unsigned p=0; p<AT32_FLASH_NPAGES; p++) erased[p]=0;
        erasures=0;
        for (unsigned i=0; i<3; i++) {
            unsigned page=sequences[s][i];
            erased[page]=0; // A save programs the page before its next erase.
            CHECK(up_progmem_eraseblock(page)==AT32_FLASH_PAGESIZE);
            CHECK(erased[page]==1);
            CHECK(addr_latch[page<512?0:1]
                  ==AT32_FLASH_BASE+page*AT32_FLASH_PAGESIZE);
        }
        CHECK(erasures==3 && g_lock==0);
    }
    CHECK(up_progmem_eraseblock(AT32_FLASH_NPAGES)==-EFAULT);
    CHECK(erasures==3 && g_lock==0);
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix='px4-at32-flash-') as directory:
        build = Path(directory)
        c_file = build / 'test.c'
        c_file.write_text(prefix + actual + tests)
        wasm = build / 'test.wasm'
        subprocess.run([args.compiler, '--target=wasm32', '-ffreestanding', '-fno-builtin', '-nostdlib',
                        '-Wall', '-Wextra', '-Werror', '-Wl,--no-entry', '-Wl,--export=run_tests',
                        str(c_file), '-o', str(wasm)], check=True)
        runner = build / 'test.js'
        runner.write_text('const fs=require("fs"); WebAssembly.instantiate(fs.readFileSync(process.argv[2])).then(({instance})=>{\n'
                          'const line=instance.exports.run_tests(); console.log(line?"FAIL C line "+line:"PASS: repeated parameter-page erase, reversed order, both banks, invalid page");\n'
                          'process.exit(line?1:0); });\n')
        subprocess.run([args.node, str(runner), str(wasm)], check=True)


if __name__ == '__main__':
    main()
