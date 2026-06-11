#!/bin/sh
#
# install-benchabi.sh — deploy the sfc7120 userspace test sources to the
# Morello board and build the *purecap-benchmark ABI* binaries natively
# there (mirrors install.sh; same sources, different ABI).
#
# Copies into the same $REMOTE_DIR layout as install.sh so both ABIs'
# binaries live side by side:
#
#   $REMOTE_DIR/sfc7120_uapi.h
#   $REMOTE_DIR/userlib/{build-benchabi.sh, test.c, bench.c,
#                        sfc7120_user.c, sfc7120_user.h}
#
# then runs ./build-benchabi.sh on the board to produce
# $REMOTE_DIR/userlib/sfctest-cb and $REMOTE_DIR/userlib/sfcbench-cb.
#
# The kernel modules (modmap.ko, sfc7120pol.ko) must already be loaded — see
# the repo CLAUDE.md. The userspace CAPIO headers (modmap.h, capio.h) must be
# on the board at $MODMAP_INC.
#
# Usage:
#   ./install-benchabi.sh
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
scp build-benchabi.sh test.c bench.c sfc7120_user.c sfc7120_user.h \
                                               "$MORELLO:$REMOTE_DIR/userlib/"

echo ""
echo "=== Building on $MORELLO (native cc, purecap-benchmark ABI) ==="
ssh "$MORELLO" "cd $REMOTE_DIR/userlib && MODMAP_INC=$MODMAP_INC sh ./build-benchabi.sh build"

echo ""
echo "=== Done ==="
echo "On the board ($MORELLO), with modmap.ko + sfc7120pol.ko loaded:"
echo "  # functional smoke test (two terminals):"
echo "  $REMOTE_DIR/userlib/sfctest-cb rx     # consumer, PF1"
echo "  $REMOTE_DIR/userlib/sfctest-cb tx     # producer, PF0"
echo "  # latency/throughput benchmark (single process, both PFs):"
echo "  $REMOTE_DIR/userlib/sfcbench-cb       # both modes, 64/1024/1518 B sweep"
echo ""
echo "Compare against the purecap binaries (sfctest / sfcbench) from"
echo "install.sh to isolate the Morello cap-branch penalty."
