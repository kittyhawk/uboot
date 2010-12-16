#
# (C) Copyright 2002
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#
debug=0
include $(OBJTREE)/include/config.mk

# CNS expects the CN kernel at 0x10; IO node has an extra trampoline
ifeq ($(KHVMM), y)
TEXT_BASE = 0
else
#TEXT_BASE = 0x10
#TEXT_BASE = 0xFFFC0000
TEXT_BASE = 0x0
endif

# An hard overload :(
PLATFORM_CPPFLAGS = -DCONFIG_PPC -D__powerpc__ -DCONFIG_4xx -ffixed-r2 \
                    -ffixed-r29 -mstring -msoft-float -Wall -Werror

ifeq ($(findstring powerpc-bgp-linux-,$(CROSS_COMPILE)),powerpc-bgp-linux-)
PLATFORM_CPPFLAGS += -mcpu=450fp2
else
PLATFORM_CPPFLAGS += -mcpu=440fp
endif

ifeq ($(debug),1)
PLATFORM_CPPFLAGS += -DDEBUG
endif

CONFIG_LWIP_TCP = n
