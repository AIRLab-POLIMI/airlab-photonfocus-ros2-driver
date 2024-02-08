set(PACKAGE_HEADER_FILES PvBase.h)
# log4cxx

set(PACKAGE_LIBRARIES
    PvPersistence
    PtUtilsLib
    PtConvertersLib
    PvAppUtils
    PvCameraBridge
    EbTransportLayerLib
    PvGenICam
    PvBase
    PvSystem
    PvDevice
    PvVirtualDevice
    EbUtilsLib
    PvStream
    PvTransmitter
    #XmlParser_gcc49_v3_3
    #NodeMapData_gcc49_v3_3
    #log4cpp_gcc49_v3_3
    #Log_gcc49_v3_3
    #MathParser_gcc49_v3_3
    #GCBase_gcc49_v3_3
    #GenApi_gcc49_v3_3
    PvSerial
    PvBuffer
    SimpleImagingLib)

set(PACKAGE_PATH /opt/jai/ebus_sdk/linux-aarch64-arm)
set(PACKAGE_PATH_GENI ${PACKAGE_PATH}/lib/genicam/bin/Linux64_ARM)

set(EBUS_SDK_INCLUDE_DIR "")
set(EBUS_SDK_INCLUDE_DIR_VAR_NAMES "")
foreach(header_file ${PACKAGE_HEADER_FILES})
    find_path(
        ${header_file}_INCLUDE_DIR
        NAMES ${header_file}
        PATHS "${PACKAGE_PATH}/include")
    list(APPEND EBUS_SDK_INCLUDE_DIR ${${header_file}_INCLUDE_DIR})
    list(APPEND EBUS_SDK_INCLUDE_DIR_VAR_NAMES ${header_file}_INCLUDE_DIR)
endforeach()

set(EBUS_SDK_LIBRARIES "")
set(EBUS_SDK_LIBRARY_VAR_NAMES "")
foreach(library ${PACKAGE_LIBRARIES})
    find_library(
        ${library}_LIBRARY
        NAMES ${library}
        PATHS "${PACKAGE_PATH}/lib" "${PACKAGE_PATH_GENI}")
    list(APPEND EBUS_SDK_LIBRARIES ${${library}_LIBRARY})
    list(APPEND EBUS_SDK_LIBRARY_VAR_NAMES ${library}_LIBRARY)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    EBUS_SDK
    FOUND_VAR EBUS_SDK_FOUND
    REQUIRED_VARS ${EBUS_SDK_INCLUDE_DIR_VAR_NAMES} ${EBUS_SDK_LIBRARY_VAR_NAMES})
mark_as_advanced(${EBUS_SDK_INCLUDE_DIR_VAR_NAMES} ${EBUS_SDK_LIBRARY_VAR_NAMES})