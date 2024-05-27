include(FindPkgConfig)
pkg_check_modules(LIBUSB libusb-1.0 IMPORTED_TARGET)
pkg_check_modules(LIBUVC libuvc IMPORTED_TARGET)

get_filename_component(HSDAOH_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

if(NOT TARGET hsdaoh::hsdaoh)
  include("${HSDAOH_CMAKE_DIR}/hsdaohTargets.cmake")
endif()
