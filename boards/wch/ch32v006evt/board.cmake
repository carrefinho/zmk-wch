# Copyright (c) 2025 Michael Hope <michaelh@juju.nz>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(minichlink)
board_runner_args(wchisp)
if(EXISTS ${ZEPHYR_BASE}/boards/common/minichlink.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/minichlink.board.cmake)
endif()
if(EXISTS ${ZEPHYR_BASE}/boards/common/wchisp.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/wchisp.board.cmake)
endif()

board_runner_args(openocd "--file-type=elf" "--cmd-reset-halt" "halt")
if(EXISTS ${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endif()
