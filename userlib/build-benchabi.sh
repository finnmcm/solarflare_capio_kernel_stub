#!/bin/sh
#
# build-benchabi.sh — cross-compile sfctest/sfcbench for the Morello
# *purecap-benchmark ABI*, or build natively on CheriBSD.
#
# The benchmark ABI is identical to purecap (same CheriABI syscalls, same
# 16-byte capabilities, CAPIO token round-trip untouched) except code
# pointers use integer branches instead of capability branches — removing
# the Morello prototype's cap-branch microarchitectural penalty. Comparing
# these binaries against the purecap ones from build.sh isolates that
# penalty from genuine capability overhead.
#
# Output: ./sfctest-cb and ./sfcbench-cb (benchmark-ABI binaries; the
# CheriBSD image activator routes them to ld-elf64cb.so.1 + /usr/lib64cb
# via the NT_CHERI_MORELLO_PURECAP_BENCHMARK_ABI ELF note).
#
# Usage:
#   ./build-benchabi.sh [build|clean|help]
#
# Env overrides:
#   CHERI_ROOT     CHERI build root (default: $HOME/cheri)
#   MODMAP_INC     dir containing modmap.h + capio.h
#                  (default: $HOME/E1000Lwip/include/netif)
#
set -e

CHERI_ROOT="${CHERI_ROOT:-$HOME/cheri}"
MODMAP_INC="${MODMAP_INC:-$HOME/E1000Lwip/include/netif}"

SDK="$CHERI_ROOT/output/morello-sdk"
CLANG="$SDK/bin/clang"
PURECAP_CFG="$SDK/bin/cheribsd-morello-purecap.cfg"
# No SDK .cfg ships for the benchmark ABI; -mabi after --config overrides
# the cfg's -mabi=purecap and the driver switches its multilib to lib64cb.
ABI_FLAGS="-mabi=purecap-benchmark"

SRCS="test.c sfc7120_user.c"
OUT="sfctest-cb"
BENCH_SRCS="bench.c sfc7120_user.c"
BENCH_OUT="sfcbench-cb"
CFLAGS="-Wall -O2 -g -I$MODMAP_INC"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

build() {
    if [ ! -f "$MODMAP_INC/modmap.h" ] || [ ! -f "$MODMAP_INC/capio.h" ]; then
        echo "Error: modmap.h / capio.h not found under $MODMAP_INC"
        echo "       set MODMAP_INC to the dir holding the userspace CAPIO headers"
        exit 1
    fi

    if [ "$(uname -s)" = "FreeBSD" ]; then
        # Native build on the Morello board: base cc is Morello clang.
        echo "Native build (CheriBSD, purecap-benchmark ABI)"
        cc $ABI_FLAGS $CFLAGS $SRCS -o "$OUT"
        cc $ABI_FLAGS $CFLAGS $BENCH_SRCS -o "$BENCH_OUT"
    else
        echo "Cross-compiling for Morello purecap-benchmark ABI"
        if [ ! -x "$CLANG" ] || [ ! -f "$PURECAP_CFG" ]; then
            echo "Error: Morello SDK not found at $SDK"
            echo "       build it with cheribuild (cheribsd-morello-purecap), or set CHERI_ROOT"
            exit 1
        fi
        "$CLANG" --config "$PURECAP_CFG" $ABI_FLAGS $CFLAGS $SRCS -o "$OUT"
        "$CLANG" --config "$PURECAP_CFG" $ABI_FLAGS $CFLAGS $BENCH_SRCS -o "$BENCH_OUT"
    fi
    echo "Build complete: $SCRIPT_DIR/$OUT $SCRIPT_DIR/$BENCH_OUT"
}

case "${1:-build}" in
    build) build ;;
    clean) rm -f "$OUT" "$BENCH_OUT"; echo "Cleaned" ;;
    help|-h|--help)
        echo "Usage: $0 [build|clean|help]"
        echo "  build   cross-compile (Linux) or native build (CheriBSD) -> ./$OUT ./$BENCH_OUT"
        echo "  clean   remove ./$OUT ./$BENCH_OUT"
        ;;
    *) echo "Usage: $0 [build|clean|help]"; exit 1 ;;
esac
