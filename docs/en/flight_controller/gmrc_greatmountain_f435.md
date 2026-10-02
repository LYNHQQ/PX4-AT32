# GMRC GREATMOUNTAINF435

<Badge type="warning" text="Experimental" />

::: warning
The direct-start application verified startup, USB communication and IMU/barometer samples on the development board.
The relocated application and USB bootloader have passed Windows builds, paired J-Link readback, USB upload/CRC and startup checks.
Real ESC telemetry, electrical reconnect, calibration and flight validation remain incomplete.
PX4 does not manufacture this autopilot, and manufacturer support for this PX4 port has not been established.
:::

The board configuration follows the [GREATMOUNTAINF435 pin reference](https://github.com/betaflight/config/blob/master/configs/GMRC/GREATMOUNTAINF435/config.h).
It targets the AT32F435VM with 4032 KiB internal flash, 384 KiB SRAM and an 8 MHz external crystal.
The clock configuration runs the Cortex-M4F at 288 MHz.
Confirm the fitted MCU marking before programming another board revision.

## Pin Assignments

| Function                | Pins                                           | PX4 Configuration                                          |
| ----------------------- | ---------------------------------------------- | ---------------------------------------------------------- |
| IMU SPI1                | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS, PA15 DRDY | ICM42688P or BMI270, probed sequentially                   |
| OSD SPI2                | PD1 SCK, PD3 MISO, PD4 MOSI, PD5 CS            | ATXXXX/MAX7456 compatible OSD                              |
| External Flash SPI3     | PC10 SCK, PC11 MISO, PC12 MOSI, PD6 CS         | JEDEC-probed M25P16, W25Q128FV or W25N01GV                 |
| I2C3                    | PC0 SCL, PC1 SDA                               | DPS310 or BMP280, addresses 0x76/0x77                      |
| UART1                   | PA9 TX, PA10 RX                                | `/dev/ttyS0`, RC serial port                               |
| UART2                   | PA2 TX, PA3 RX                                 | `/dev/ttyS1`, TEL1                                         |
| UART3                   | PC4 TX, PC5 RX                                 | `/dev/ttyS2`, GPS1                                         |
| UART4                   | PA0 TX, PA1 RX                                 | `/dev/ttyS3`, TEL2                                         |
| UART8                   | PC2 TX, PC3 RX                                 | `/dev/ttyS4`, console at 115200 baud                       |
| USB                     | PA11 D−, PA12 D+                               | CDC ACM, `/dev/ttyACM0`                                    |
| Outputs 1–4             | PB6, PB7, PB8, PB9                             | TIM4 channels 1–4                                          |
| Outputs 5–8             | PC6, PC7, PC8, PC9                             | TIM3 channels 1–4                                          |
| Battery Voltage/Current | PB0 ADC1 channel 8, PB1 ADC1 channel 9         | 3.3 V ADC reference                                        |
| LEDs                    | PD15, PD14                                     | Assumed active low; polarity and colours need verification |
| Beeper                  | PB11                                           | Active low GPIO, intended for an active buzzer             |

The reference does not specify connector types, supply ratings, or a purchase link.
Refer to the board schematic and silkscreen for electrical connections.
PD2 and the PB10 LED strip output are left unused by this port.

## Build and Programming

The port uses native AT32 drivers and startup code in the NuttX submodule, with PX4 architecture support and board configuration in the main repository.
Include the NuttX source changes when integrating this port, as for other PX4 hardware architectures.

Build the target in a supported PX4 development environment:

```sh
make gmrc_greatmountain-f435_default
```

This branch uses a USB-only PX4 bootloader built with the following target:

```sh
make gmrc_greatmountain-f435_bootloader
```

The bootloader occupies the first 64 KiB at `0x08000000`–`0x0800ffff`.
The matching application starts at `0x08010000` and has a 3960 KiB linker region ending before the parameter pages.
The protocol reports an application capacity of 4,055,040 bytes, equal to the 4032 KiB physical Flash minus the 64 KiB bootloader and 8 KiB parameter reservation.
Program the bootloader BIN at `0x08000000` and the matching application BIN at `0x08010000`, then verify both complete readbacks.
A direct-start application linked at `0x08000000` is incompatible with this layout.

The bootloader uses PX4 protocol revision 5 and experimental board ID 0, matching the application package.
Its USB product string is `PX4 BL GMRC GREATMOUNTAINF435`, distinguishing it from the running application.
`reboot -b` requests the bootloader through the retained RTC backup-register signature.
An invalid application also causes the bootloader to remain available.
The relocated application has passed the bootloader handoff, `reboot -b`, USB upload/CRC and system-reset checks described below.

Confirm the MCU and the required 384 KiB SRAM option before running this target on another board.
The connected development board uses `EOPB0=0xfd02`; firmware updates do not require changing USD.
Preserve all option settings and avoid a complete raw USD write.

The linker reserves `0x083ee000`–`0x083effff`, the final two 4 KiB erase pages, for redundant parameter storage.
Preserve these pages when updating firmware if parameters must be retained.
The board early script uses `/fs/microsd/parameters_backup.bson` as its optional nonempty backup path.

## Configuration

The default build includes the multicopter controllers, EKF2, MAVLink, USB CDC ACM, common serial RC support, PWM/DShot outputs, battery monitoring and OSD.
The IMU orientation is translated from the reference with the Betaflight/PX4 coordinate conventions taken into account.
An external GNSS receiver and serial RC protocol must be configured for the connected hardware.

The board applies `IMU_GYRO_RATEMAX=400` as a default in `rc.board_sensors`, after airframe defaults and before IMU startup.
This avoids the multicopter airframe increasing the implicit default to 800 Hz.
Explicitly saved parameter values take precedence.

Initial battery settings are `BAT1_V_DIV=11` and `BAT1_A_PER_V=100`, converted from the reference scaling.
Check voltage and current calibration against measurements on the actual board.

### USB

USB CDC ACM is enabled, with `SYS_USB_AUTO=2` starting MAVLink on `/dev/ttyACM0` after the host enumerates the USB device.
The USB controller uses a 48 MHz clock derived from the 288 MHz PLL clock divided by 6.
PA9 and PA10 remain UART1 pins; USB VBUS sensing and the OTG ID pin are disabled.
Without a separate VBUS input, CDC attachment is allowed continuously and the driver waits for host enumeration.

The configured VID/PID is `0x3185:0x0035`, using the V6X default pair requested for this port.
The manufacturer string is `GMRC` and the product string is `GMRC GREATMOUNTAINF435`.
Electrical unplug/reconnect, host sleep/reboot, other hosts and long-term transport reliability still require validation.
Manufacturer authorisation for this VID/PID has not been verified.

### External Flash

Startup reads the SPI3 JEDEC ID before selecting the native NuttX M25P16, W25Q128FV or W25N01GV driver.
Unknown IDs are rejected, and the driver supplies the capacity and erase geometry.
The detected flash is registered as `/dev/mtd0`, and startup attempts to mount an existing LittleFS filesystem at `/fs/microsd` for flight logs and other PX4 files.

Startup never automatically formats the filesystem.
The existing NuttX W25N driver manages NAND ECC and bad-block mappings; its initialisation can erase spare blocks and update mappings.
Back up existing NAND contents before first running this firmware.
A failed mount is nonfatal.
After backing up the contents, explicitly create a PX4 filesystem:

```sh
mklittlefs /dev/mtd0 /fs/microsd
```

This command erases data to create LittleFS and mounts it immediately.
Reboot afterwards so services initialise with storage available.
Complete filesystem writes, retention, wear and power-loss recovery still need hardware validation.
Without mounted storage, the board defaults to the RAM dataman backend.
Missions and fences then do not survive a reboot.
With LittleFS mounted, the default dataman backend uses persistent files; an explicitly saved `SYS_DM_BACKEND` selection takes precedence.
The default `SDLOG_MAX_SIZE=1` MiB allows log rotation on the smallest supported flash.
Increase it only within the fitted capacity and leave room for another file.
The 2 MiB M25P16 is suitable only for short bench logs: the logger requires at least 1 MiB free before starting.

### Motor Outputs

PWM at 400 Hz remains the default.
Configure each group with its timer parameter, then reboot:

| Parameter       | Outputs | Hardware Timer |
| --------------- | ------- | -------------- |
| `PWM_MAIN_TIM0` | 1–4     | TIM4           |
| `PWM_MAIN_TIM1` | 5–8     | TIM3           |

| Protocol               | Parameter Value |
| ---------------------- | --------------- |
| PWM 400 Hz (default)   | `400`           |
| OneShot                | `-1`            |
| DShot150               | `-5`            |
| DShot300               | `-4`            |
| DShot600               | `-3`            |
| Bidirectional DShot150 | `-8`            |
| Bidirectional DShot300 | `-7`            |
| Bidirectional DShot600 | `-6`            |

For example, set both timer parameters to `-6` to select bidirectional DShot600 on all eight outputs.
Bidirectional DShot requires compatible ESC firmware.
DShot600 specifies a wire bit rate of 600 kbit/s, independently of the output frame frequency.
Telemetry capture is implemented for all eight outputs; each timer's channel 4 uses channel 3's indirect input capture internally.
ESC timing, actual eRPM/EDT and signal levels require external equipment.

Addressable LED strips and CAN are not enabled.
Verify mounting orientation, voltage/current calibration, physical USB reconnection, output waveforms, actual ESC telemetry, persistent storage and worst-case memory under vehicle load before flight testing.

## Memory and Verification

High-frequency code is selected by the board linker script for the zero-wait Flash window and the SRAM `.ramfunc` section.
The physical zero-wait Flash window ends at `0x08040000`; the 64 KiB bootloader leaves 192 KiB for the application in this window.
The application retains the `Ekf` core class and the IMU, rate-control, native IRQ/DMA and scheduler groups, rather than moving the entire EKF2 module to slower Flash.
Other EKF2 wrappers and helper classes remain in the ordinary Flash region.
The linker asserts the physical boundary at `0x08040000`, independent of the relocated application origin.
Native startup follows the STM32/NuttX pattern, copying `.ramfunc` from Flash to SRAM before board initialisation.
The AT32 implementation also uses DSB/ISB to synchronise writes before instruction execution from SRAM.
The SRAM selection includes native DShot and timer control, the uORB publication path and selected MAVLink and arithmetic functions, with a 24 KiB linker limit.
The board enables the existing NuttX ARMv7-M `memset` and `strcmp` implementations in the zero-wait Flash region.
SPI1 DMA uses static 512-byte receive and transmit buffers; the PX4 DMA allocation-pool counter does not include these buffers.
Transfers exceeding the DMA buffer or transfer-count limit use the existing polling path.
Larger IMU FIFO batches can therefore cost more CPU time; the default 400 Hz gyro rate limit keeps a normal ICM42688P batch within the DMA buffer.

Register-model regression checks cover PWM/OneShot output-mask changes and preservation of independent timer and DShot ownership.
The board enables native BCH support required by the existing PX4 MTD layer.

Native NuttX changes are limited to UART8 availability, SRAM startup copying, USB device register setup, SPI DMA buffer and completion handling, the 8 MHz/288 MHz clock sequence and internal Flash bank status selection.
Only DMA1 and USB device mode are enabled; the general external Flash drivers are unmodified.
UART break transmission is disabled; the selected RC, GNSS and MAVLink drivers do not require it.
Startup supports a complete MCU/system reset or the controlled bootloader handoff.
The handoff switches to HICK, disables PLL, keeps HEXT enabled for the RTC, resets the USB controller and clears pending USB, SysTick and PendSV interrupts before entering application startup.

For PLL frequencies above 108 MHz, the clock sequence retains the 20 NOP instructions required by reference manual section 4.1.5.
They execute after selecting PLL and before polling `SCLKSTS`, with automatic step mode still enabled.
The [manufacturer SDK clock-switch function](https://github.com/ArteryTek/AT32F435_437_Firmware_Library/blob/master/libraries/drivers/src/at32f435_437_crm.c#L743) includes the same wait through `DUMMY_NOP()`.
The compile-time frequency condition leaves lower-frequency configurations unchanged.
The 8 MHz RTC source is divided by eight to match the existing 1 MHz prescaler input.

The original HICK enable handling and Flash erase-address `modifyreg32()` are retained.
The documented protection of the active HICK source and zero reset value returned by write-only Flash address registers make the proposed replacements unnecessary for this startup sequence.
The Flash correction instead selects the status register using an absolute bank boundary.
USB setup leaves reserved `GCCFG` bits 18 and 19 at zero; this register correction alone does not establish the cause of earlier USB failures.

### Current Performance Verification

The Windows application build and complete ELF/BIN/PX4 comparison passed on 2 October 2026.
The current BIN is 1,339,021 bytes, with SHA256 `17f552dace3ba47706b0c512153522b89c0db37a9da09b93d52eb896bc879c7a`.
The ELF SHA256 is `9787cd030253c638d27e9587448228593bad2fb8c67da21b25866350ea4f0a29`.
The unchanged 46,644-byte bootloader has SHA256 `3fdc221bbb0f52765712cee9514d4d6eefbd7ea65ca49edff5694049ea7c0393`.
Paired J-Link programming and complete readbacks passed, preserving USD and both parameter pages byte for byte.
The target was left running with `VTOR=0x08010000`, SWD pins at AF0 and zero CFSR/HFSR.

This optimisation changes only this board's linker layout, DMA-pool configuration and layout regression.
It retains the saved quad configuration, four assigned motors, bidirectional DShot600, 400 Hz gyro/rate/output chain, approximately 200 Hz IMU integration and EKF2 work-queue execution, and approximately 100 Hz EKF updates.
Control algorithms, floating-point options and shared module sources are unchanged.

Low-frequency native DShot/timer initialisation and status functions, totalling 2,772 bytes, now remain in ordinary Flash.
The SRAM space is used by `ControlAllocator::Run`, actuator publication and the sequential-desaturation implementation, including their linked thunks.
Moving the 3,612-byte Ekf constructor/destructor group out of the zero-wait window allows the complete AttitudeControl library, ImuDownSampler and `EstimatorInterface::setIMUData` to run there.
The actual ELF passes complete-function address-range checks for the existing and new hot groups, including cold-code exclusion.

| Queue                 | Previous `d1928bea` CPU | Current `17f552da` CPU |
| --------------------- | ----------------------- | ---------------------- |
| `rate_ctrl`           | 19.43–19.44%            | 15.72%                 |
| `INS0`                | 27.26–27.32%            | 19.75%                 |
| `nav_and_controllers` | 30.10–30.12%            | 29.02–29.07%           |
| `SPI1`                | About 4.35%             | About 4.23%            |

Each image was sampled in two 15-second scheduler-counter windows at the same saved settings and Hold boot mode.
Live SWD reads can skew task accounting, which also includes interrupt time charged to the interrupted task.
Background USB traffic, uptime and estimator state differ; these measurements do not isolate the contribution of each relocated function or establish worst-case flight load.
The new windows measured approximately 9.26% idle, while a separate `top once` with greater USB activity measured 5.35% idle.
The five-second module elapsed-counter window measured about 97 microseconds per allocation cycle, compared with about 172 microseconds on the previous image.
Elapsed counters include preemption and are not exclusive CPU time.
The unchanged Hold fallback still retries flight-task selection without valid horizontal position or RC input, so its navigation load remains an outstanding shared-module issue.

The ICM42688P retains 400 Hz FIFO service and 20-sample gyro/accelerometer batches, with zero register, transfer, empty, overflow and missed-DRDY errors.
DPS310 communication errors and both timer DMA error counters remain zero.
The measured window recorded no new VehicleIMU data gaps or missed EKF2 IMU messages.
SPI1 sampling observed 154 RX and 138 TX transfers with enabled DMA, nonzero changing counts and matching peripheral/buffer addresses.
The maximum observed transfer count was 404 bytes within each static 512-byte buffer; the SPI clock remains 18 MHz.
A separate 1,000-sample aligned-word check measured RX semaphore counts in `[-1, 0]` and TX counts in `[0, 1]`.
A 30-second USB run completed all ten accepted requests with no parsing errors or sequence gaps, finite IMU/barometer data and a device-to-host elapsed-time ratio of 1.00008.
Bounded NSH parameter, sensor, DShot, work-queue, allocator and `top once` commands succeeded.
A read-only external Flash test returned 2,048 bytes from `/dev/mtd0`; LittleFS remains unmounted.
The unchanged linked ARM `memset` and `strcmp` implementations passed 121,016 instruction-emulation cases and both deliberately broken entry controls were rejected.

#### SRAM, Heap And Stacks

The MCU has 384 KiB SRAM; the 24 KiB limit is this board's executable-code budget, not its total SRAM capacity.
The native NuttX startup copies `.ramfunc` from Flash before early initialisation, followed by DSB/ISB barriers.
All 24,232 executable bytes, including 1,016 bytes of linker veneers, match the Flash load image at runtime, leaving 344 bytes in that budget.
Running PC samples include 1,636 SRAM addresses out of 8,000 observations and specifically include the relocated allocation functions.
This demonstrates execution in SRAM, rather than an unbiased statistical CPU profile.
The zero-wait Flash window has 688 bytes remaining.

Heap allocations and task stacks already use SRAM.
The native heap extends from `0x2000cde8`, after static data/BSS and the idle stack, to `0x20060000`; task creation allocates stack storage from this heap.
A high-frequency work queue calls functions in multiple code regions, so its name does not identify a single execution region.
Moving more code into SRAM consumes the same memory needed by task stacks, subscriptions and other dynamic allocations.

The removed 5 KiB board DMA pool had no linked allocation consumers in this configuration: FAT DMA allocation is disabled, storage uses LittleFS, and SPI/DShot use their own static DMA buffers.
Its pool initialisation and unused granule allocator configuration were removed together.
This reclaims memory, rather than directly reducing driver CPU time.
Future FAT/SDIO DMA configurations must reassess pool and alignment requirements.
Static SRAM use falls from 56,704 to 51,944 bytes, increasing the initial heap from 335,744 to 340,504 bytes, a gain of 4,760 bytes after the code-layout changes.
One running allocator snapshot showed 24,624 bytes free, a 320,664-byte usage peak and a 24,576-byte largest free block.
The rate-control stack high-water mark was 2,344 of 3,128 bytes, leaving 784 bytes; SPI1, navigation and INS stack observations left 576, 936 and 2,104 bytes respectively.
These bare-board observations do not establish worst-case flight stack, heap or CPU margin.
Actual ESC eRPM/EDT and output waveforms remain unverified without connected ESCs and measurement equipment.

### Earlier 400 Hz Performance Verification

The Windows application build and its complete ELF, BIN and PX4 package comparison passed on 2 October 2026.
The BIN is 1,339,205 bytes, with SHA256 `d1928beab427a055e7ddda883325590d13fa70a8e53333b4240af7f7dca0bad5`.
The ELF SHA256 is `afbb3624958cf50cffd37d51487847414a6501e9c646aa5286f4aeb485466308`.
The unchanged 46,644-byte bootloader has SHA256 `3fdc221bbb0f52765712cee9514d4d6eefbd7ea65ca49edff5694049ea7c0393`.
Paired J-Link programming and complete readbacks passed, with USD and both parameter pages unchanged.
The board was left running with `VTOR=0x08010000` and zero CFSR/HFSR.

The retained quad configuration uses `SYS_AUTOSTART=4001`, four assigned motors and bidirectional DShot600 on timer group 0.
Group 1 remains unassigned.
No ESC is connected.
The implicit airframe default previously increased the gyro, PID, allocation and DShot chain to approximately 800 Hz.
Applying the board default after airframe selection restores approximately 400 Hz without overwriting saved rate choices.
IMU integration and EKF2 run at approximately 200 Hz.

The AT32 bidirectional capture completion now leaves the pins released with pull-ups until the next frame restores output.
This avoids restoring every pin twice per frame.
The extracted production-driver regression passed 1,051,851 assertions, including consecutive frames, missing responses and reinitialisation.
The two linked ARM libc functions passed 121,016 Cortex-M4 Thumb instruction-emulation cases for alignment, tails, byte values, string ordering, register preservation and memory boundaries.
Both deliberately broken function-entry controls were rejected.
Instruction emulation verifies function semantics, rather than hardware timing.

| Image And Configuration                    | Gyro/Rate Chain | Rate-Control Queue CPU | SPI1 Queue CPU | Idle CPU |
| ------------------------------------------ | --------------- | ---------------------- | -------------- | -------- |
| `4981f4c4` baseline                        | About 800 Hz    | 48.94%                 | 9.74%          | 0.00%    |
| `fdf5b733` board default and expanded SRAM | About 400 Hz    | 21.27%                 | 5.38%          | 1.50%    |
| `d1928bea` native ARM libc optimisation    | About 400 Hz    | 19.48%                 | 4.35%          | 2.44%    |

These are live scheduler-counter windows, approximately five seconds for the first two images and ten seconds for the current image.
Task accounting includes interrupt time charged to the interrupted task, and live SWD reads can introduce sampling skew.
The first comparison changes both rate and memory placement; it does not isolate the SRAM benefit.
The second retains the 400 Hz configuration, but background traffic and estimator state can still differ.
The rate-control queue includes angular-velocity processing, PID, allocation and DShot, rather than PID alone.
Current elapsed-cycle means were approximately 26 microseconds for PID, 172 microseconds for allocation and 213 microseconds for DShot.
Module elapsed counters include preemption and must not be added as exclusive CPU percentages.
The navigation and INS queues still use approximately 30.19% and 27.61% respectively, so the overall CPU margin remains small.

The current boot-mode default is `COM_FLTMODE_BOOT=4` (Hold).
Live state checks found the vehicle disarmed, without valid horizontal position/velocity or RC input, and the flight-mode manager running its Descend fallback task.
The shared flight-task selection then retries Auto, manual acceleration and manual altitude tasks before returning to Descend on successive cycles.
This reconstructs tasks in the existing fixed union and repeats subscription lookups; it is not evidence of a task-allocation heap leak.
Two seconds of aligned word sampling observed transitions through None, Auto, ManualAcceleration, ManualAltitudeSmoothVel and Descend, confirming intermediate retries that a single endpoint snapshot would miss.
A temporary Stabilized-mode comparison on the same image retained the 400 Hz chain and measured 19.45% rate-control, 15.51% navigation, 27.55% INS and 7.75% idle CPU.
The flight-mode manager reported no active flight task in that mode.
Hold was restored afterwards, with Descend active again and the vehicle still disarmed.
This identifies invalid-position Hold fallback as additional bench load, rather than an optimisation of the flight-control algorithms.
Neither the saved parameters nor the boot-mode default were changed.

The ICM42688P services its FIFO at 400 Hz, publishing 20 gyro/accelerometer samples per batch with zero register, transfer, empty, overflow and missed-DRDY errors.
The initial FIFO reset count is one.
The DPS310 communication error count is zero.
The measured window recorded no new VehicleIMU gyro or accelerometer data gaps and no timer DMA errors.
Read-only SPI1 sampling observed 142 RX and 144 TX transfers with DMA enabled, nonzero changing counts, matching peripheral/buffer addresses and the expected DMAMUX requests.
The maximum observed RX count was 404 bytes, within the static 512-byte buffer, and the SPI clock remained 18 MHz.
A separate 1,000-sample aligned-word check observed RX semaphore counts in `[-1, 0]` and TX counts at zero.
The earlier bytewise structure read produced torn semaphore values during interrupts; those values are not treated as real counter corruption.
A 30-second USB test completed all ten accepted requests with no parsing errors or sequence gaps, finite IMU/barometer data and a device-to-host elapsed-time ratio of 0.99939.
Bounded NSH parameter, sensor, DShot, work-queue, allocator and `top once` commands returned successfully.

All 23,816 executable SRAM bytes match their Flash load image, leaving 760 bytes within the 24 KiB section limit.
Running PC samples include 1,660 SRAM addresses out of 8,000 observations; this establishes execution there, rather than an unbiased CPU profile.
Static SRAM use is 56,704 bytes, with an initial linker heap of 335,744 bytes.
One allocator snapshot showed 19,712 bytes free, a 320,968-byte usage peak and a 19,712-byte largest free block.
The zero-wait Flash window has 1,640 bytes remaining.
These bare-board measurements do not establish worst-case stack, heap or CPU margin in flight.
The rate-control stack high-water mark was 2,376 of 3,128 bytes; it does not establish worst-case flight stack demand.
Final read-only checks retained SWD pin AF0, zero fault registers and byte-identical USD and parameter pages.

### Known Shared-Driver Limitations

The shared DShot caller does not consume capture-ready results for offline channels.
On this bare board, ready bits remain latched and telemetry processing repeats on subsequent output cycles.
The resulting receive-event counter does not establish successful ESC packets or eRPM.
Actual eRPM, EDT and electrical output waveforms still require connected ESCs and measurement equipment.

The shared MAVLink shell retains a mutex/70-byte-pipe wait cycle observed on the 800 Hz image.
The receiver held the shell mutex while waiting for space in the output pipe, while the sender waited for that mutex.
Bounded commands succeeded on the current image, but lower load does not repair that shared blocking contract.
These shared-source corrections remain separate from the board and AT32 adaptation.
The repeated flight-task reconstruction under invalid-position Hold also belongs to the shared flight-mode manager.
The adaptation retains the default boot mode and leaves that separate retry optimisation outstanding.

### Earlier Relocated-Application Verification

The relocated Windows application build completed successfully.
Its BIN is 1,338,453 bytes, with SHA256 `6681f89280782ec8d6719cc2035eee0048cedd7d5536f5699baecb2a7e06ed26`.
The Windows bootloader build completed successfully.
Its BIN is 46,644 bytes, with SHA256 `3fdc221bbb0f52765712cee9514d4d6eefbd7ea65ca49edff5694049ea7c0393`.
This rebuild differs from the read-back image only in the embedded `PX4BL` build identity string.
Complete paired J-Link readbacks match both BIN images.
The original PX4 uploader completed three USB erase/program/CRC/reboot runs: the application in 34 seconds, a temporary Bank2 boundary probe in 40 seconds, and the original application again in 34 seconds.
All three runs returned successfully.
The temporary package retained the application prefix and placed a 16-byte marker at `0x08200000`; readback confirmed the marker and the erased remainder of the first Bank2 page.
After restoring the original application, all 4096 bytes of this page were erased again, and the full bootloader and application readbacks still matched their original BIN images.
Each upgrade preserved USD and both parameter pages byte for byte.
Later output tests deliberately saved and restored parameters; the upgrade-preservation checks do not describe those separate saves.

Read-only protocol identification confirmed revision 5, board ID/revision 0, capacity 4,055,040 bytes and the MCU UID matching SWD.
Windows enumerated the bootloader product as `PX4 BL GMRC GREATMOUNTAINF435`.
`reboot -b` entered the bootloader, and the original uploader returned to the application with `VTOR=0x08010000` and zero CFSR/HFSR.
After restoring the default output settings and resetting, a final 30-second USB run completed all 10 accepted requests without parsing errors or sequence gaps; the device-to-host elapsed-time ratio was 0.99975.

The relocated application repeats the ICM42688P 400 Hz, 20-sample gyro/accelerometer and DPS310 barometer checks with zero transfer, register, overflow, missed-DRDY and barometer communication errors.
ADC channels 8, 9 and 16 publish samples, and a read-only `/dev/mtd0` test returned 2048 bytes; LittleFS remains unmounted.
All eight bidirectional DShot600 channels initialise with mask `0xff`, telemetry enabled and zero timer DMA errors.
The early DShot snapshot records 48 cycles and zero bidirectional receive events, so it does not establish capture activity or actual eRPM/EDT without an ESC.
Both PWM groups were saved at 400 Hz, all eight output functions set to zero and the gyro rate limit restored to 400 Hz; reset checks confirmed these settings.

All 15,344 executable SRAM bytes match the Flash load image.
Of 2000 running PC samples, 394 were in SRAM; this proves execution there but is not a statistical CPU profile.
SWD retains AF0, and CFSR/HFSR are zero.
The allocator snapshot showed 48,872 bytes available and a 299,304-byte usage peak; it does not establish the largest free block or worst-case vehicle memory margin.
The relocated zero-wait Flash section has 2220 bytes remaining.
No airframe is selected and the sensors remain uncalibrated.
The direct-start evidence below is retained for the unchanged board drivers and does not verify this branch's bootloader, application addresses or upload path.

### Verified Direct-Start Application

The direct-start Windows build was programmed and read back through J-Link on 2 October 2026.
The BIN is 1,338,357 bytes, with SHA256 `f141716437598a31b29e9407f6aa7976afc66dd2bfb1e054f87aaf81427f884b`.
The ELF, BIN and PX4 package payload match, and 111 generated native driver copies match their source files.
Disassembly confirms 20 consecutive NOP instructions between selecting PLL and polling `SCLKSTS`.
Programming preserved USD and both parameter pages.
Startup passed after programming and additional system resets.
Two 30-second USB runs each completed 10 accepted requests without parsing errors or sequence gaps.
The final run drained pre-existing host receive data before measuring device time; its device-to-host time ratio was 1.00035.
An earlier undrained attempt included two stale packets and failed the timing assertion, while requests and packet integrity passed.

The SPI change is limited to three corrections: checking DMA count and bounce-buffer bounds before copying, mapping the RX bounce buffer independently of TX, and waiting for TX completion after a successful RX wait.
Oversized transfers use the existing polling path.
The timeout and additional DMA request/reset changes from the earlier implementation have been removed.
An extracted-source host regression passed all 25 cases covering bounds, RX-only and TX-only transfers, 8/16-bit words, aliasing, wait failures and polling fallback.

Live J-Link sampling at 400 Hz observed both SPI1 DMA channels enabled with changing transfer counts and matching peripheral/buffer addresses.
Of 1,500 observations, 114 caught RX DMA enabled and 100 caught TX DMA enabled.
Sampled RX/TX semaphore counts remained within `[-1, 0]` and `[0, 1]` respectively.
These register observations verify DMA activity, rather than a CPU performance improvement.
A bounded `top once` sample at 900.145 seconds uptime on the direct-start image showed 22.150% idle, 4.925% SPI1, 1.885% rate control, 21.789% INS0 and 22.499% USB MAVLink CPU usage.
This snapshot is not a controlled comparison of optimisation gains.

That direct-start image passed ICM42688P and DPS310 sampling and SRAM verification.
At the restored default 400 Hz FIFO service, the IMU publishes 20 samples per batch with no transfer, register, overflow or missed-DRDY errors.
DPS310 communication errors are zero.
A saved 250 Hz test configuration produced 32-sample, 644-byte FIFO batches, exceeding the 512-byte DMA buffer; valid samples and zero driver errors confirmed polling fallback.
The 400 Hz parameter was then saved, reset and verified.
Both PWM groups remain at 400 Hz with all eight output functions unassigned.

All 15,344 executable SRAM bytes match the Flash load image.
SWD pins retain AF0, and CFSR/HFSR are zero.
Static SRAM use is 48,256 bytes, with an initial linker heap of 344,192 bytes.
An allocator snapshot showed 57,400 bytes available and a 287,344-byte usage peak.
These counters do not establish the largest free block or worst-case vehicle memory margin.
The zero-wait Flash section has 1,004 bytes remaining.
SPI1 retains its static 512-byte DMA buffers and 18 MHz bus clock.
The earlier DShot, ADC, external Flash, RTC and PC sampling checks below were not repeated on this revision.

### Earlier Bench Verification

The checks below were completed on the earlier Windows firmware on 2 October 2026 after reducing the native NuttX changes to seven files.
They apply to that image and must not be treated as repeated measurements on later revisions.
The tested BIN is 1,338,701 bytes, with SHA256 `164a57615d2b9352f28773b9a312c4f96186ee6b1d6b44784b5b11a4653eefe1`.
The ELF, BIN and PX4 package payload match.

| Check          | Result                                                                                                                          |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Programming    | Complete application readback matches; USD and both parameter pages unchanged by programming                                    |
| USB            | Two 30-second runs, each with 10 accepted requests, no parsing errors or sequence gaps                                          |
| ICM42688P      | 400 Hz FIFO service, valid gyro/accelerometer samples, no transfer, register, overflow or missed-DRDY errors                    |
| DPS310         | Valid pressure/temperature samples on I2C3 at 0x76, no communication errors                                                     |
| SPI1 DMA       | Both DMA1 channels observed enabled with changing remaining counts; 404-byte IMU transfers and static buffer addresses verified |
| SRAM           | All 15,344 executable bytes match the Flash load image; running PC samples include SRAM addresses                               |
| Clocks         | 288 MHz system clock, 48 MHz USB clock, 1 MHz RTC input; RTC advanced two seconds over a two-second host interval               |
| SWD            | PA13/PA14 retain AF0; CFSR and HFSR are zero                                                                                    |
| ADC            | Channels 8, 9 and 16 publish samples; voltage/current calibration remains required                                              |
| External Flash | `/dev/mtd0` returned 2,048 bytes in a read-only test; LittleFS was not mounted                                                  |
| DShot          | All eight bidirectional DShot600 channels initialise; both timer DMA error counters remain zero                                 |
| Parameters     | Test settings survive reset; original 400 Hz PWM and unassigned output functions restored and verified after reset              |

The bare board has no ESC connected, so bidirectional capture reports offline channels and response errors.
These results do not verify actual eRPM, EDT or electrical output waveforms.
The OSD driver was not running in the final default configuration, and display validation remains outstanding.
No airframe is selected (`SYS_AUTOSTART=0`), and the sensors remain uncalibrated.

Static SRAM use is 48,256 bytes, leaving an initial linker heap of 344,192 bytes.
One running allocator snapshot showed 57,152 bytes available and a 299,648-byte usage peak; this does not establish the largest free block or worst-case flight margin.
The zero-wait Flash section has 660 bytes remaining.
A bounded `top once` sample at 192 seconds uptime showed 23.91% idle, 5.10% SPI1, 1.88% rate control and 21.82% INS0 CPU usage.
This is a bench snapshot rather than a controlled performance comparison.

These checks apply to the image identified above and do not replace electrical, calibration, persistent-filesystem or flight validation.

The current shared MAVLink shell can block during sustained, high-volume console output, and the shared ATXXXX driver has a display-address error above 255.
These require separate shared-driver corrections; use bounded console commands when validating this port and check OSD character positioning with a video display.

Upstream board registration still requires an assigned unique board ID, manufacturer-authorised USB identifiers, product/connector information and vehicle flight logs demonstrating basic operation.
The experimental target does not yet satisfy those acceptance requirements.

Additional physical checks remain open for the BOOT/NRST wiring, genuine power-on resets, the UART8 console connection, visible LED polarity/colours and beeper sound.
The reference does not establish those details.
On the VMT device, ROM USART2 uses PD5/PD6, conflicting with the board's OSD and external Flash chip selects; application UART2 on PA2/PA3 is not the same recovery interface.
ROM recovery requires verified wiring and mode-entry information before any attempt to change option settings.
