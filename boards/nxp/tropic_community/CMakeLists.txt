#
# Copyright 2026 CogniPilot Foundation
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

# Same Winbond W25Q64 8MB flash chip and boot header requirements as the
# upstream "teensy41" board (same physical module) - unlike
# mr_vmu_tropic's flexspi config, board.c is NOT nested inside this
# condition since it also brings up the status LEDs, which have nothing
# to do with the XIP boot header.
if(CONFIG_NXP_IMXRT_BOOT_HEADER)
  zephyr_compile_definitions(XIP_BOOT_HEADER_ENABLE=1)
  zephyr_compile_definitions(XIP_BOOT_HEADER_DCD_ENABLE=1)
  zephyr_compile_definitions(BOARD_FLASH_SIZE=CONFIG_FLASH_SIZE*1024)
  zephyr_library_sources(xip/flexspi_nor_config.c)
endif()

zephyr_library_sources(board.c)
