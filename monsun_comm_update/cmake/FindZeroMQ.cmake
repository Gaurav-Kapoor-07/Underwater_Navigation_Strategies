find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_ZeroMQ REQUIRED libzmq)

find_path(ZeroMQ_INCLUDE_DIR
    NAMES zmq.h
    HINTS ${PC_ZeroMQ_INCLUDE_DIRS}
)

find_library(ZeroMQ_LIBRARY
    NAMES libzmq.so libzmq.dylib libzmq.dll
    HINTS ${PC_ZeroMQ_LIBRARY_DIRS}
)

set(ZeroMQ_VERSION ${PC_ZeroMQ_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZeroMQ
    DEFAULT_MSG
    ZeroMQ_LIBRARY
    ZeroMQ_INCLUDE_DIR
)

mark_as_advanced(ZeroMQ_INCLUDE_DIR ZeroMQ_LIBRARY)

set(ZeroMQ_LIBRARIES ${ZeroMQ_LIBRARY})
set(ZeroMQ_INCLUDE_DIRS ${ZeroMQ_INCLUDE_DIR})
