# CheriBsdSolarflare7120 — out-of-tree CAPIO kernel driver stub for the
# Solarflare SFN7000-series (Huntington / EF10) NIC.
#
# Layout mirrors ~/CheriBsdE1000: cross-compile via CROSS_COMPILE=1 on Linux,
# native make on CheriBSD.

KMOD   = sfc7120pol
SRCS   = sfc7120.c sfc7120_mcdi.c sfc7120_mmio.c sfc7120_tables.c capio.c \
         device_if.h bus_if.h pci_if.h opt_platform.h ofw_bus_if.h

.if defined(CROSS_COMPILE) && ${CROSS_COMPILE} == "1"

EXPORT_SYMS = YES

CHERI_ROOT 		= ${HOME}/cheri
CHERI_SOURCE 	= ${CHERI_ROOT}/cheribsd
CHERI_LLVM_OUT 	= ${CHERI_ROOT}/output/morello-sdk/bin

CFLAGS += -DCROSS_COMPILE=${CROSS_COMPILE}
.ifndef CHERIBSD_SOURCE_PATH
.error "CHERIBSD_SOURCE_PATH must be set to your CheriBSD source directory"
.endif

CHERI_HOST_TOOLS = ${CHERI_ROOT}/build/cheribsd-morello-purecap-build/${CHERIBSD_SOURCE_PATH}/arm64.aarch64c/tmp/legacy/bin

SYSDIR = ${CHERI_SOURCE}/sys

MACHINE 		= arm64
MACHINE_ARCH 	= aarch64c
TARGET			= aarch64c
TARGET_ARCH		= aarch64c

CC 	= ${CHERI_LLVM_OUT}/clang
CXX = ${CHERI_LLVM_OUT}/clang++
LD	= ${CHERI_LLVM_OUT}/ld.lld
OBJCOPY = ${CHERI_LLVM_OUT}/llvm-objcopy
XARGS = ${CHERI_HOST_TOOLS}/xargs
AWK = ${CHERI_HOST_TOOLS}/awk

CFLAGS += --target=aarch64-unknown-freebsd14
CFLAGS += -march=morello+c64 -mabi=purecap
CFLAGS += -DCHERI -DCHERI_CAPREVOKE

.else

SYSDIR = /usr/src/sys
CFLAGS += -I${HOME}/CheriModmap

.endif

.include <bsd.kmod.mk>
