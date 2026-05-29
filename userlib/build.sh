#!/bin/sh
#
# build.sh — cross-compile the sfc7120 userspace TX/RX test (sfctest) for the
# Morello board, or build natively on CheriBSD.
#
# Output: ./sfctest (Morello purecap binary)
#
# Usage:
#   ./build.sh [build|clean|help]
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

SRCS="test.c sfc7120_user.c"
OUT="sfctest"
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
        # Native build on the Morello board: cc already targets purecap.
        echo "Native build (CheriBSD)"
        cc $CFLAGS $SRCS -o "$OUT"
    else
        echo "Cross-compiling for Morello purecap"
        if [ ! -x "$CLANG" ] || [ ! -f "$PURECAP_CFG" ]; then
            echo "Error: Morello SDK not found at $SDK"
            echo "       build it with cheribuild (cheribsd-morello-purecap), or set CHERI_ROOT"
            exit 1
        fi
        "$CLANG" --config "$PURECAP_CFG" $CFLAGS $SRCS -o "$OUT"
    fi
    echo "Build complete: $SCRIPT_DIR/$OUT"
}

case "${1:-build}" in
    build) build ;;
    clean) rm -f "$OUT"; echo "Cleaned" ;;
    help|-h|--help)
        echo "Usage: $0 [build|clean|help]"
        echo "  build   cross-compile (Linux) or native build (CheriBSD) -> ./$OUT"
        echo "  clean   remove ./$OUT"
        ;;
    *) echo "Usage: $0 [build|clean|help]"; exit 1 ;;
esac
