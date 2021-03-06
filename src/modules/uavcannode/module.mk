############################################################################
#
#   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
#   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
#           David Sidrane <david)s5@nscdg.com>
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

#
# UAVCAN <--> Flow
#

# Needed for Weak Simboles for IRQ
DEFAULT_VISIBILITY=default


MODULE_COMMAND = uavcannode

MAXOPTIMIZATION = -O3

MODULE_STACKSIZE = 1024


# Main
SRCS += uavcannode_main.cpp       \


#
# libuavcan
#
include $(PX4_LIB_DIR)uavcan/libuavcan/include.mk
# Use the relitive path to keep the genrated files in the BUILD_DIR
SRCS += $(subst  $(PX4_MODULE_SRC),../../,$(LIBUAVCAN_SRC))
INCLUDE_DIRS += $(LIBUAVCAN_INC)
# Since actual compiler mode is C++11, the library will default to UAVCAN_CPP11, but it will fail to compile
# because this platform lacks most of the standard library and STL. Hence we need to force C++03 mode.
override EXTRADEFINES := $(EXTRADEFINES) \
-DUAVCAN_CPP_VERSION=UAVCAN_CPP03 \
-DUAVCAN_NO_ASSERTIONS \
-DUAVCAN_MEM_POOL_BLOCK_SIZE=48 \
-DUAVCAN_MAX_NETWORK_SIZE_HINT=16 \
-DUAVCAN_STM32_TIMER_NUMBER=2
#
# libuavcan drivers for STM32
#
include $(PX4_LIB_DIR)uavcan/libuavcan_drivers/stm32/driver/include.mk
# Use the relitive path to keep the genrated files in the BUILD_DIR
SRCS += $(subst  $(PX4_MODULE_SRC),../../,$(LIBUAVCAN_STM32_SRC))
INCLUDE_DIRS += $(LIBUAVCAN_STM32_INC)
override EXTRADEFINES += -DUAVCAN_STM32_BAREMETAL -DUAVCAN_STM32_NUM_IFACES=1

#
# Invoke DSDL compiler
#
$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR) $(PX4_MODULE_SRC)/modules/uavcannode/dsdl/uavcan/threedr))
INCLUDE_DIRS += dsdlc_generated


#
# Using boot app shared structures
#
INCLUDE_DIRS += $(PX4_BOOTLOADER_BASE)include

override EXTRADEFINES += -DAPP_VERSION_MAJOR=$(UAVCANBLID_SW_VERSION_MAJOR) -DAPP_VERSION_MINOR=$(UAVCANBLID_SW_VERSION_MINOR) \
-DHW_UAVCAN_NAME=$(UAVCANBLID_NAME) -DHW_VERSION_MAJOR=$(UAVCANBLID_HW_VERSION_MAJOR) -DHW_VERSION_MINOR=$(UAVCANBLID_HW_VERSION_MINOR)



