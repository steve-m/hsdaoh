find_path(CRCFAST_INCLUDE_DIR
    NAMES libcrc_fast.h
)

find_library(CRCFAST_LIBRARY
    NAMES crc_fast
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CRCFAST
    REQUIRED_VARS CRCFAST_LIBRARY CRCFAST_INCLUDE_DIR
)

if(CRCFAST_FOUND)
    add_library(CRCFAST::CRCFAST UNKNOWN IMPORTED)
    set_target_properties(CRCFAST::CRCFAST PROPERTIES
        IMPORTED_LOCATION "${CRCFAST_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CRCFAST_INCLUDE_DIR}"
    )
endif()
