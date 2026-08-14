# Copyright (c) 2025 Bootlin
# Copyright (c) 2026 carrefinho
# SPDX-License-Identifier: Apache-2.0

board_runner_args(minichlink "--dt-flash=y")
board_runner_args(wlink "--chip=CH32V20X")
board_runner_args(wchisp)
if(EXISTS ${ZEPHYR_BASE}/boards/common/minichlink.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/minichlink.board.cmake)
endif()
if(EXISTS ${ZEPHYR_BASE}/boards/common/wlink.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/wlink.board.cmake)
endif()
if(EXISTS ${ZEPHYR_BASE}/boards/common/wchisp.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/wchisp.board.cmake)
endif()
