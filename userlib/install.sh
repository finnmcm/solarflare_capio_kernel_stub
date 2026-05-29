#!/bin/sh
#
# install.sh — deploy the sfc7120 userspace test sources to the Morello board
# and build them natively there (mirrors build_lwip_apps.sh).
#
# Copies the sources preserving the repo layout so sfc7120_user.h's
# "../sfc7120_uapi.h" include resolves:
#
#   $REMOTE_DIR/sfc7120_uapi.h
#   $REMOTE_DIR/userlib/{build.sh, test.c, sfc7120_user.c, sfc7120_user.h}
#
# then runs ./build.sh on the board to produce $REMOTE_DIR/userlib/sfctest.
#
# The kernel modules (modmap.ko, sfc7120pol.ko) must already be loaded — see
# the repo CLAUDE.md. The userspace CAPIO headers (modmap.h, capio.h) must be
# on the board at $MODMAP_INC.
#
# Usage:
#   ./install.sh
#
# Env overrides:
#   MORELLO     ssh target for the board   (default: root@cheri)
#   REMOTE_DIR  destination dir on board   (default: /root/sfc7120_test)
#   MODMAP_INC  CAPIO headers on the board (default: /root/E1000Lwip/include/netif)
#
set -e

MORELLO="${MORELLO:-root@cheri}"
REMOTE_DIR="${REMOTE_DIR:-/root/sfc7120_test}"
MODMAP_INC="${MODMAP_INC:-/root/E1000Lwip/include/netif}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Copying sources to $MORELLO:$REMOTE_DIR ==="
ssh "$MORELLO" "mkdir -p $REMOTE_DIR/userlib"
scp ../sfc7120_uapi.h                          "$MORELLO:$REMOTE_DIR/"
scp build.sh test.c sfc7120_user.c sfc7120_user.h \
                                               "$MORELLO:$REMOTE_DIR/userlib/"

echo ""
echo "=== Building on $MORELLO (native cc) ==="
ssh "$MORELLO" "cd $REMOTE_DIR/userlib && MODMAP_INC=$MODMAP_INC sh ./build.sh build"

echo ""
echo "=== Done ==="
echo "On the board ($MORELLO), with modmap.ko + sfc7120pol.ko loaded:"
echo "  # terminal 1 (consumer, PF1):"
echo "  $REMOTE_DIR/userlib/sfctest rx"
echo "  # terminal 2 (producer, PF0):"
echo "  $REMOTE_DIR/userlib/sfctest tx"
