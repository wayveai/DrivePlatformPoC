## CMake configuration file

set(LIBNAME vibrante_Xlibs)

# installation prefix
get_filename_component (CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component (_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/" ABSOLUTE)

# include directory
#
# Newer versions of CMake set the INTERFACE_INCLUDE_DIRECTORIES property
# of the imported targets. It is hence not necessary to add this path
# manually to the include search path for targets.
set (${LIBNAME}_LIBRARY_DIR "${_INSTALL_PREFIX}/lib")
set (${LIBNAME}_INCLUDE_DIR "${_INSTALL_PREFIX}/include")

# alias for default import target to be compatible with older CMake package configurations
set (${LIBNAME}_LIBRARIES "${LIBNAME}")

# --------
# modifications based on PDK changes
set(_VIBRANTE_XLIBS_LIBRARIES X11 Xrandr Xinerama Xi Xcursor)

# import targets
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 3.1.3)
    # Support for old cmake that did not support INTERFACE libraries
    set (${LIBNAME}_LIBRARIES ${_VIBRANTE_XLIBS_LIBRARIES})
    include_directories(${${LIBNAME}_INCLUDE_DIR})
else()
    add_library(${LIBNAME} INTERFACE)

    # include directories
    target_include_directories(${LIBNAME} INTERFACE ${${LIBNAME}_INCLUDE_DIR})

    # add all libraries with absolute paths
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
    foreach(LIB ${_VIBRANTE_XLIBS_LIBRARIES})
        set(FOUND_LIB "FOUND_LIB-NOTFOUND")
        find_library(FOUND_LIB ${LIB}
                    HINTS ${${LIBNAME}_LIBRARY_DIR}
                    NO_DEFAULT_PATH)
        if(FOUND_LIB)
            target_link_libraries(${LIBNAME} INTERFACE ${FOUND_LIB})
            message(STATUS "Found vibrante_Xlib: ${FOUND_LIB}")
        else()
            target_link_libraries(${LIBNAME} INTERFACE ${LIB})
        endif()
    endforeach()
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)

    set (${LIBNAME}_LIBRARIES ${LIBNAME})
endif()

# unset private variables
unset (_INSTALL_PREFIX)
