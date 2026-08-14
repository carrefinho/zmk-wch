if(CONFIG_SOC_FAMILY_CH32VX)
  # hal_ch32fun.h shim + vendored ch32fun vendor headers. Vendored because the
  # stock zephyr west.yml pins hal_wch to the pre-ch32fun layout (ch32v003fun/
  # only); this keeps the module independent of that pin. Source: ch32fun @
  # dd3855ea624b (MIT, see include/ch32fun/LICENSE).
  zephyr_include_directories(include include/ch32fun)
endif()
add_subdirectory(drivers)
