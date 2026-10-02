/****************************************************************************
 * GREATMOUNTAINF435 linker layout
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The AT32F435VM has 4032 KiB internal flash and 384 KiB contiguous SRAM.
 * The PX4 bootloader reserves the first 64 KiB; the application starts at
 * 0x08010000.
 * Reserve the last two 4 KiB erase pages for redundant parameter storage.
 */

MEMORY
{
    flash (rx) : ORIGIN = 0x08010000, LENGTH = 3960K
    sram (rwx) : ORIGIN = 0x20000000, LENGTH = 384K
}

OUTPUT_ARCH(arm)

ENTRY(__start)		/* treat __start as the anchor for dead code stripping */
EXTERN(_vectors)	/* force the vectors to be included in the output */

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
EXTERN(abort)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)

		/* EOPB0=2 provides 384 KiB SRAM and 256 KiB zero-wait Flash.
		 * Prioritize estimator, IMU and rate control code in that Flash window.
		 * The bootloader uses 64 KiB of this window. Keep the Ekf core and
		 * other archive groups here without a per-function list.
		 */
		. = ALIGN(4);
		_szw_hot = ABSOLUTE(.);
		/* Constructors/destructors run at start/stop; keep the running core hot. */
		*libmodules__ekf2.a:*(.text._ZN3Ekf[0-9]* .text._ZNK3Ekf*
		                    .text._ZN14ImuDownSampler* .text._ZNK14ImuDownSampler*
		                    .text._ZN18EstimatorInterface10setIMUData*)
		*libAttitudeControl.a:*(.text .text.*)
		*liboutput_predictor.a:*(.text .text.*)
		*libyaw_estimator.a:*(.text .text.*)
		*libbias_estimator.a:*(.text .text.*)
		*libvehicle_imu.a:*(.text .text.*)
		*libvehicle_angular_velocity.a:*(.text .text.*)
		*libdrivers_gyroscope.a:*(.text .text.*)
		*libdrivers_accelerometer.a:*(.text .text.*)
		*libdrivers__imu__invensense__icm42688p.a:*(.text .text.*)
		*libsensor_calibration.a:*(.text .text.*)
		*libmodules__mc_rate_control.a:*(.text .text.*)
		*libRateControl.a:*(.text .text.*)
		*libdrivers__device.a:*(.text .text.*)
		*libpx4_work_queue.a:*(.text .text.*)
		*libarch_hrt.a:*(.text .text.*)
		*libarch.a:*(.text .text.*)

		/* Shared wakeup, memory and serial paths serve all work queues and
		 * telemetry. Keep runtime member families here rather than the whole
		 * NuttX libc/filesystem, which would displace the estimator.
		 */
		_szw_common = ABSOLUTE(.);
		*libsched.a:sem_*.o(.text .text.*)
		*libsched.a:sched_*.o(.text .text.*)
		*libsched.a:sig_*.o(.text .text.*)
		*libsched.a:clock_*.o(.text .text.*)
		*libsched.a:wd_*.o(.text .text.*)
		*libsched.a:irq_*.o(.text .text.*)
		*libsched.a:pthread_mutex.o(.text .text.*)
		*libc.a:arch_mem*.o(.text .text.*)
		*libc.a:arch_strcmp.o(.text .text.*)
		*libc.a:lib_mem*.o(.text .text.*)
		*libc.a:lib_str[cln]*.o(.text .text.*)
		*libc.a:lib_mutex.o(.text .text.*)
		*libc.a:sem_*.o(.text .text.*)
		*libc.a:lib_errno.o(.text .text.*)
		*libdrivers.a:serial.o(.text .text.*)
		*libdrivers.a:serial_io.o(.text .text.*)
		*libdrivers.a:cdcacm.o(.text .text.*)
		_ezw_common = ABSOLUTE(.);
		_ezw_hot = ABSOLUTE(.);

		/* Leave boot, reconfiguration and status reporting in ordinary Flash,
		 * freeing SRAM for the 400 Hz allocation path below.
		 */
		*libarch_dshot.a:*(.text.up_dshot_init* .text.up_dshot_arm* .text.up_bdshot_status*)
		*libarch_io_pins.a:*(.text.io_timer_*allocate_* .text.io_timer_*init*
		                    .text.io_timer_set_pwm_rate* .text.up_pwm_servo_*init*)

		/* Leave the bounded RAM group for .ramfunc below. MAVLink is a unity
		 * object, so claim its selected send families before its cold remainder.
		 */
		EXCLUDE_FILE (*libuORB.a:Publication.cpp.obj
		              *libuORB.a:Subscription*.cpp.obj
		              *libuORB.a:uORBDeviceNode.cpp.obj
		              *libuORB.a:uORBManager.cpp.obj
		              *libuORB.a:uORBDeviceMaster.cpp.obj
		              *libarch_dshot.a:*
		              *libarch_io_pins.a:*
		              *libmodules__control_allocator.a:*
		              *libControlAllocation.a:*
		              *libuorb_msgs.a:uORBTopics.cpp.obj
		              *libgcc.a:_arm_addsubdf3.o
		              *libgcc.a:_arm_muldivdf3.o
		              *libgcc.a:_arm_cmpdf2.o
		              *libgcc.a:_arm_unorddf2.o
		              *libgcc.a:_arm_truncdfsf2.o
		              *libm.a:libm_a-w_sqrt.o
		              *libm.a:libm_a-e_sqrt.o
		              *libmodules__mavlink.a:*) *(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		EXCLUDE_FILE (*libuorb_msgs.a:uORBTopics.cpp.obj
		              *libm.a:libm_a-w_sqrt.o
		              *libm.a:libm_a-e_sqrt.o) *(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
	} > flash

	ASSERT(_ezw_hot <= 0x08040000,
	       "AT32 zero-wait hot code exceeds 256 KiB")

	/* Native NuttX copies this bounded executable group before board init.
	 * Keep early clock/FPU/UART code in Flash. Data and task stacks remain
	 * after this range so the native _ebss-based idle stack/heap cannot overlap.
	 */
	.ramfunc : ALIGN(8) {
		_sramfuncs = ABSOLUTE(.);
		*(.ramfunc .ramfunc.*)
		*libuORB.a:Publication.cpp.obj(.text .text.*)
		*libuORB.a:Subscription*.cpp.obj(.text .text.*)
		*libuORB.a:uORBDeviceNode.cpp.obj(.text .text.*)
		*libuORB.a:uORBManager.cpp.obj(.text .text.*)
		/* Keep the timer DMA/turnaround path and repeated topic lookup in SRAM.
		 * Select the native architecture archives without altering shared drivers.
		 */
		*libuORB.a:uORBDeviceMaster.cpp.obj(.text._ZN4uORB12DeviceMaster19getDeviceNodeLocked*)
		*libarch_dshot.a:*(.text .text.*)
		*libarch_io_pins.a:*(.text .text.*)
		*libmodules__control_allocator.a:*(.text.*N16ControlAllocator3Run*
		                                 .text._ZN16ControlAllocator25publish_actuator_controls*)
		*libControlAllocation.a:*(.text._ZN39ControlAllocationSequentialDesaturation*)
		*libmodules__mavlink.a:*(.text.crc_accumulate*
		                       .text.*mavlink_finalize_message*
		                       .text._mav_finalize_message_chan_send*
		                       .text.mavlink_start_uart_send*
		                       .text.mavlink_end_uart_send*
		                       .text.mavlink_send_uart_bytes*
		                       .text._ZN7Mavlink*get_free_tx_buf*
		                       .text._ZN7Mavlink*send_start*
		                       .text._ZN7Mavlink*send_finish*
		                       .text._ZN7Mavlink*send_bytes*
		                       .text._ZN7Mavlink*update_rate_mult*
		                       .text._ZN13MavlinkStream*update*
		                       .text._ZN7Mavlink9task_main*
		                       .text.*MavlinkStream*8get_sizeEv
		                       .text.*MavlinkStream*12get_size_avgEv
		                       .text.*MavlinkStream*10const_rateEv)
		/* Copy the topic pointer table with its accessor. Metadata and future
		 * unrelated read-only additions remain in the cold Flash remainder.
		 */
		*libuorb_msgs.a:uORBTopics.cpp.obj(.text._Z12get_orb_meta6ORB_ID
		                                .rodata._ZL16uorb_topics_list)
		*libgcc.a:_arm_addsubdf3.o(.text .text.*)
		*libgcc.a:_arm_muldivdf3.o(.text .text.*)
		*libgcc.a:_arm_cmpdf2.o(.text .text.*)
		*libgcc.a:_arm_unorddf2.o(.text .text.*)
		*libgcc.a:_arm_truncdfsf2.o(.text .text.*)
		*libm.a:libm_a-w_sqrt.o(.text .text.* .rodata .rodata.*)
		*libm.a:libm_a-e_sqrt.o(.text .text.* .rodata .rodata.*)
		. = ALIGN(8);
		_eramfuncs = ABSOLUTE(.);
	} > sram AT > flash
	_framfuncs = LOADADDR(.ramfunc);

	ASSERT((_eramfuncs - _sramfuncs) <= 24K,
	       "AT32 RAM functions exceed 24 KiB")
	ASSERT((_sramfuncs & 7) == 0 && (_eramfuncs & 7) == 0 &&
	       (_framfuncs & 7) == 0, "AT32 RAM functions must be 8-byte aligned")

	.ramfunc_cold : {
		*libmodules__control_allocator.a:*(.text .text.*)
		*libControlAllocation.a:*(.text .text.*)
		*libuORB.a:uORBDeviceMaster.cpp.obj(.text .text.*)
		*libmodules__mavlink.a:*(.text .text.*)
		*libuorb_msgs.a:uORBTopics.cpp.obj(.text .text.* .rodata .rodata.*)
		_etext = ABSOLUTE(.);
	} > flash

	/*
	 * Init functions (static constructors and the like)
	 */
        .init_section : {
                _sinit = ABSOLUTE(.);
                KEEP(*(.init_array .init_array.*))
                _einit = ABSOLUTE(.);
        } > flash


	.ARM.extab : {
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash
	ASSERT(_sdata >= _eramfuncs, "AT32 data overlaps RAM functions")
	ASSERT(LOADADDR(.data) == _eronly, "AT32 data load does not match startup")
	ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(flash) + LENGTH(flash),
	       "AT32 load image overlaps parameter pages")

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8);
		_ebss = ABSOLUTE(.);
	} > sram

	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
