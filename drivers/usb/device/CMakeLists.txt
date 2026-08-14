zephyr_library_named(zmk_wch_usb_dc)
zephyr_library_sources_ifdef(CONFIG_USB_DC_WCH usb_dc_wch.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/usb/device)
