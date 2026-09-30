# Copyright 2026 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

# Same physical module (Teensy 4.1) and bootloader as the upstream
# "teensy41" board, so it uses the same teensy-loader-cli based runner
# (via the Teensy's own USB HalfKay bootloader) rather than mr_vmu_tropic's
# jlink/pyocd/linkserver debug-probe runners - this board has no onboard
# SWD/JTAG debug connector.
board_set_flasher_ifnset(teensy)
board_runner_args(teensy "--mcu=TEENSY41")

include(${ZEPHYR_BASE}/boards/common/teensy.board.cmake)
