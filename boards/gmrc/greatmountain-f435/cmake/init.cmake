############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

if(NOT PX4_BOARD STREQUAL "gmrc_greatmountain-f435" OR NOT PX4_PLATFORM STREQUAL "nuttx")
	return()
endif()

if(CMAKE_VERSION VERSION_LESS 3.19)
	message(FATAL_ERROR "GREATMOUNTAINF435 board hooks require CMake 3.19 or newer")
endif()

if(CONFIG_ARMV7M_MEMSET AND CMAKE_C_COMPILER_ID STREQUAL "GNU")
	# PX4 passes C flags to the NuttX Make assembler. Match the native ARM
	# toolchain option required by its optimized Thumb memset implementation.
	string(APPEND CMAKE_C_FLAGS " -Wa,-mimplicit-it=always")
endif()

function(gmrc_f435_quote_cxx_include directory)
	# Targets inherit the original NuttX option when they are created. Quote the
	# forced header only in this board's configure, including paths with spaces.
	get_property(targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
	foreach(target IN LISTS targets)
		get_target_property(options ${target} COMPILE_OPTIONS)
		if(options)
			string(REPLACE
				"-include${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/include/cxx/abs_overloads.hpp"
				"SHELL:-include \"${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/include/cxx/abs_overloads.hpp\""
				options "${options}")
			set_property(TARGET ${target} PROPERTY COMPILE_OPTIONS "${options}")
		endif()
	endforeach()

	get_property(children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
	foreach(child IN LISTS children)
		gmrc_f435_quote_cxx_include("${child}")
	endforeach()
endfunction()

function(gmrc_f435_finalize)
	gmrc_f435_quote_cxx_include("${CMAKE_SOURCE_DIR}")

	# SCRIPT_PREFIX is set in the platforms/nuttx directory scope and is not
	# visible in this deferred call; derive the same value from the board label.
	set(script_prefix "")
	if("${PX4_BOARD_LABEL}" MATCHES "^bootloader")
		set(script_prefix "bootloader_")
	elseif("${PX4_BOARD_LABEL}" STREQUAL "canbootloader")
		set(script_prefix "canbootloader_")
	elseif(NOT "${PX4_BOARD_LINKER_PREFIX}" STREQUAL "")
		set(script_prefix "${PX4_BOARD_LINKER_PREFIX}-")
	endif()

	if(TARGET px4)
		# Editing only the linker script must relink the firmware as well.
		set_property(TARGET px4 APPEND PROPERTY LINK_DEPENDS
			"${NUTTX_CONFIG_DIR}/scripts/${script_prefix}script.ld")
	endif()

	# RESET_ONLY already selects the native reset path. Its unused dump helpers
	# need no shared-source change or suppression on unrelated translation units.
	foreach(target px4_layer px4_kernel_layer)
		if(TARGET ${target})
			set_property(SOURCE "${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/common/board_crashdump.c"
				TARGET_DIRECTORY ${target} APPEND PROPERTY COMPILE_OPTIONS -Wno-unused-function)
		endif()
	endforeach()
endfunction()

# px4 is created after the existing board-init include.
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL gmrc_f435_finalize)
