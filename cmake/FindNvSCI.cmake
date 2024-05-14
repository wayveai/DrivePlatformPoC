################################################################################
#
# Notice
# ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS"
# NVIDIA MAKES NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR
# OTHERWISE WITH RESPECT TO THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED
# WARRANTIES OF NONINFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR
# PURPOSE.
#
# NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences
# of use of such information or for any infringement of patents or other rights
# of third parties that may result from its use. No license is granted by
# implication or otherwise under any patent or patent rights of NVIDIA
# CORPORATION & AFFILIATES. No third party distribution is allowed unless
# expressly authorized by NVIDIA. Details are subject to change without notice.
# This code supersedes and replaces all information previously supplied. NVIDIA
# CORPORATION & AFFILIATES products are not authorized for use as critical
# components in life support devices or systems without express written approval
# of NVIDIA CORPORATION & AFFILIATES.
#
# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this material and related documentation without an express
# license agreement from NVIDIA CORPORATION or its affiliates is strictly
# prohibited.
#
################################################################################

# FindNvSCI
# ---------
#
# Finds the NVIDIA Software Communications Interface (SCI) libraries (NvSCI).
#
# The following components are supported:
#
# * ``NvSciBuf``: The NvSciBuf library,
# * ``NvSciEvent``: The NvSci Event Service (NvSciEvent) library.
# * ``NvSciIpc``: The NvSci Inter-Process Communication (NvSciIpc) library.
# * ``NvSciStream``: The NvSciStream library.
# * ``NvSciSync``: The NvSciSync library.
#
# If no ``COMPONENTS`` are specified, all components are assumed.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``NvSCI::NvSciBuf``
#   The NvSciBuf library. Target defined if component NvSciBuf is found.
# ``NvSCI::NvSciEvent``
#   The NvSciEvent library. Target defined if component NvSciEvent is found.
# ``NvSCI::NvSciIpc``
#   The NvSciIpc library. Target defined if component NvSciIpc is found.
# ``NvSCI::NvSciStream``
#   The NvSciStream library. Target defined if component NvSciStream is found.
# ``NvSCI::NvSciSync``
#   The NvSciSync library. Target defined if component NvSciSync is found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``NvSCI_FOUND``
#   True if the system has the NvSCI libraries.
# ``NvSCI_INCLUDE_DIRS``
#   Include directories needed to use NvSCI.
# ``NvSCI_LIBRARIES``
#   Libraries needed to link to NvSCI.
# ``NvSCI_NvSciBuf_INCLUDE_DIRS``
#   Include directories needed to use NvSciBuf.
# ``NvSCI_NvSciBuf_LIBRARIES``
#   Libraries needed to link to NvSciBuf.
# ``NvSCI_NvSciEvent_INCLUDE_DIRS``
#   Include directories needed to use NvSciEvent.
# ``NvSCI_NvSciEvent_LIBRARIES``
#   Libraries needed to link to NvSciEvent.
# ``NvSCI_NvSciIpc_INCLUDE_DIRS``
#   Include directories needed to use NvSciIpc.
# ``NvSCI_NvSciIpc_LIBRARIES``
#   Libraries needed to link to NvSciIpc.
# ``NvSCI_NvSciStream_INCLUDE_DIRS``
#   Include directories needed to use NvSciStream.
# ``NvSCI_NvSciStream_LIBRARIES``
#   Libraries needed to link to NvSciStream.
# ``NvSCI_NvSciSync_INCLUDE_DIRS``
#   Include directories needed to use NvSciSync.
# ``NvSCI_NvSciSync_LIBRARIES``
#   Libraries needed to link to NvSciSync.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``NvSCI_INCLUDE_DIR``
#   The directory containing ``nvscierror.h``.
# ``NvSCI_LIBRARY_DIR``
#   The directory containing the libraries ``libnvscibuf.so``,
#   ``libnvscicommon.so``, ``libnvscievent.so``, ``libnvsciipc.so``,
#   ``libnvscistream.so``, and ``libnvscisync.so``.
# ``NvSCI_NvSciBuf_LIBRARY``
#   The path to ``libnvscibuf.so``.
# ``NvSCI_NvSciCommon_LIBRARY``
#   The path to ``libnvscicommon.so``.
# ``NvSCI_NvSciEvent_LIBRARY``
#   The path to ``libnvscievent.so``.
# ``NvSCI_NvSciIpc_LIBRARY``
#   The path to ``libnvsciipc.so``.
# ``NvSCI_NvSciStream_LIBRARY``
#   The path to ``libnvscistream.so``.
# ``NvSCI_NvSciSync_LIBRARY``
#   The path to ``libnvscisync.so``.

if(NvSCI_LIBRARY_DIR)
  set(_NVSCI_FIND_LIBRARY_HINTS "${NvSCI_LIBRARY_DIR}")
else()
  set(_NVSCI_FIND_LIBRARY_HINTS)
endif()
set(_NVSCI_FIND_PATH_HINTS)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^(aarch64-linux-gnu|aarch64-unknown-nto-qnx)$")
  if(VIBRANTE_PDK)
    list(APPEND _NVSCI_FIND_LIBRARY_HINTS "${VIBRANTE_PDK}/lib-target")
    list(APPEND _NVSCI_FIND_PATH_HINTS "${VIBRANTE_PDK}/include")
  endif()
endif()

list(APPEND _NVSCI_FIND_LIBRARY_HINTS
  "/usr/local/driveworks/targets/${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}/lib"
)
list(APPEND _NVSCI_FIND_PATH_HINTS
  "/usr/local/driveworks/targets/${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}/include/nvsci"
)

find_path(NvSCI_INCLUDE_DIR
  NAMES nvscierror.h
  HINTS ${_NVSCI_FIND_PATH_HINTS}
  DOC "Path to the directory containing the header file nvscierror.h."
)

unset(_NVSCI_FIND_PATH_HINTS)

set(_NVSCI_NVSCIBUF_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvscibuf${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciBuf_LIBRARY
  NAMES "${_NVSCI_NVSCIBUF_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCIBUF_SONAME}."
)

set(_NVSCI_NVSCICOMMON_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvscicommon${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciCommon_LIBRARY
  NAMES "${_NVSCI_NVSCICOMMON_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCICOMMON_SONAME}."
)

set(_NVSCI_NVSCIEVENT_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvscievent${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciEvent_LIBRARY
  NAMES "${_NVSCI_NVSCIEVENT_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCIEVENT_SONAME}."
)

set(_NVSCI_NVSCIIPC_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvsciipc${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciIpc_LIBRARY
  NAMES "${_NVSCI_NVSCIIPC_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCIIPC_SONAME}."
)

set(_NVSCI_NVSCISTREAM_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvscistream${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciStream_LIBRARY
  NAMES "${_NVSCI_NVSCISTREAM_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCISTREAM_SONAME}."
)

set(_NVSCI_NVSCISYNC_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}nvscisync${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvSCI_NvSciSync_LIBRARY
  NAMES "${_NVSCI_NVSCISYNC_SONAME}"
  HINTS ${_NVSCI_FIND_LIBRARY_HINTS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
  DOC "Path to the shared library file ${_NVSCI_NVSCISYNC_SONAME}."
)

unset(_NVSCI_FIND_LIBRARY_HINTS)

find_package(Threads QUIET MODULE REQUIRED)

include(FindPackageHandleStandardArgs)

if(NvSCI_FIND_COMPONENTS)
  set(_NVSCI_REQUIRED_VARS NvSCI_INCLUDE_DIR)

  foreach(_NVSCI_COMPONENT ${NvSCI_FIND_COMPONENTS})
    if(_NVSCI_COMPONENT STREQUAL NvSciBuf)
      if(NvSCI_FIND_REQUIRED_NvSciBuf)
        list(APPEND _NVSCI_REQUIRED_VARS
          NvSCI_NvSciBuf_LIBRARY
          NvSCI_NvSciCommon_LIBRARY
          NvSCI_NvSciIpc_LIBRARY
        )
      endif()
      if(NvSCI_INCLUDE_DIR AND NvSCI_NvSciBuf_LIBRARY)
        set(NvSCI_NvSciBuf_FOUND ON)
      endif()
    elseif(_NVSCI_COMPONENT STREQUAL NvSciEvent)
      if(NvSCI_FIND_REQUIRED_NvSciEvent)
        list(APPEND _NVSCI_REQUIRED_VARS
          NvSCI_NvSciEvent_LIBRARY
          NvSCI_NvSciIpc_LIBRARY
        )
      endif()
      if(NvSCI_INCLUDE_DIR AND NvSCI_NvSciEvent_LIBRARY)
        set(NvSCI_NvSciEvent_FOUND ON)
      endif()
    elseif(_NVSCI_COMPONENT STREQUAL NvSciIpc)
      if(NvSCI_FIND_REQUIRED_NvSciIpc)
        list(APPEND _NVSCI_REQUIRED_VARS NvSCI_NvSciIpc_LIBRARY)
      endif()
      if(NvSCI_INCLUDE_DIR AND NvSCI_NvSciIpc_LIBRARY)
        set(NvSCI_NvSciIpc_FOUND ON)
      endif()
    elseif(_NVSCI_COMPONENT STREQUAL NvSciStream)
      if(NvSCI_FIND_REQUIRED_NvSciStream)
        list(APPEND _NVSCI_REQUIRED_VARS
          NvSCI_NvSciBuf_LIBRARY
          NvSCI_NvSciCommon_LIBRARY
          NvSCI_NvSciIpc_LIBRARY
          NvSCI_NvSciStream_LIBRARY
          NvSCI_NvSciSync_LIBRARY
        )
      endif()
      if(NvSCI_INCLUDE_DIR AND NvSCI_NvSciStream_LIBRARY)
        set(NvSCI_NvSciStream_FOUND ON)
      endif()
    elseif(_NVSCI_COMPONENT STREQUAL NvSciSync)
      if(NvSCI_FIND_REQUIRED_NvSciSync)
        list(APPEND _NVSCI_REQUIRED_VARS
          NvSCI_NvSciBuf_LIBRARY
          NvSCI_NvSciCommon_LIBRARY
          NvSCI_NvSciIpc_LIBRARY
          NvSCI_NvSciSync_LIBRARY
        )
      endif()
      if(NvSCI_INCLUDE_DIR AND NvSCI_NvSciSync_LIBRARY)
        set(NvSCI_NvSciSync_FOUND ON)
      endif()
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _NVSCI_REQUIRED_VARS)

  find_package_handle_standard_args(NvSCI
    FOUND_VAR NvSCI_FOUND
    REQUIRED_VARS ${_NVSCI_REQUIRED_VARS}
    HANDLE_COMPONENTS
  )

  unset(_NVSCI_REQUIRED_VARS)
else()
  find_package_handle_standard_args(NvSCI
    FOUND_VAR NvSCI_FOUND
    REQUIRED_VARS
      NvSCI_INCLUDE_DIR
      NvSCI_NvSciBuf_LIBRARY
      NvSCI_NvSciCommon_LIBRARY
      NvSCI_NvSciEvent_LIBRARY
      NvSCI_NvSciIpc_LIBRARY
      NvSCI_NvSciStream_LIBRARY
      NvSCI_NvSciSync_LIBRARY
  )
endif()

if(NvSCI_FOUND)
  set(NvSCI_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
  mark_as_advanced(NvSCI_INCLUDE_DIR)

  if(NvSCI_LIBRARY_DIR)
    mark_as_advanced(NvSCI_LIBRARY_DIR)
  endif()

  set(NvSCI_LIBRARIES)

  if(NvSCI_NvSciCommon_LIBRARY)
    set(NvSCI_NvSciCommon_LIBRARIES "${NvSCI_NvSciCommon_LIBRARY}")
    mark_as_advanced(NvSCI_NvSciCommon_LIBRARY)
    list(APPEND NvSCI_LIBRARIES
      "${NvSCI_NvSciCommon_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )

    if(NOT TARGET NvSCI::NvSciCommon)
      add_library(NvSCI::NvSciCommon SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciCommon PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciCommon_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCICOMMON_SONAME}"
        INTERFACE_LINK_LIBRARIES Threads::Threads
      )
    endif()
  endif()

  if(NvSCI_NvSciIpc_LIBRARY)
    set(NvSCI_NvSciIpc_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
    set(NvSCI_NvSciIpc_LIBRARIES
      "${NvSCI_NvSciIpc_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )
    mark_as_advanced(NvSCI_NvSciIpc_LIBRARY)
    list(APPEND NvSCI_LIBRARIES "${NvSCI_NvSciIpc_LIBRARY}")

    if(NOT TARGET NvSCI::NvSciIpc)
      add_library(NvSCI::NvSciIpc SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciIpc PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciIpc_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCIIPC_SONAME}"
        INTERFACE_LINK_LIBRARIES Threads::Threads
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvSCI_INCLUDE_DIR}"
      )
    endif()
  endif()

  if(NvSCI_NvSciBuf_LIBRARY)
    set(NvSCI_NvSciBuf_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
    set(NvSCI_NvSciBuf_LIBRARIES
      "${NvSCI_NvSciBuf_LIBRARY}"
      "${NvSCI_NvSciCommon_LIBRARY}"
      "${NvSCI_NvSciIpc_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )
    mark_as_advanced(NvSCI_NvSciBuf_LIBRARY)
    list(APPEND NvSCI_LIBRARIES "${NvSCI_NvSciBuf_LIBRARY}")

    if(NOT TARGET NvSCI::NvSciBuf)
      add_library(NvSCI::NvSciBuf SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciBuf PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciBuf_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCIBUF_SONAME}"
        INTERFACE_LINK_LIBRARIES
          "NvSCI::NvSciCommon;NvSCI::NvSciIpc;Threads::Threads"
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvSCI_INCLUDE_DIR}"
      )
    endif()
  endif()

  if(NvSCI_NvSciEvent_LIBRARY)
    set(NvSCI_NvSciEvent_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
    set(NvSCI_NvSciEvent_LIBRARIES
      "${NvSCI_NvSciEvent_LIBRARY}"
      "${NvSCI_NvSciIpc_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )
    mark_as_advanced(NvSCI_NvSciEvent_LIBRARY)
    list(APPEND NvSCI_LIBRARIES "${NvSCI_NvSciEvent_LIBRARY}")

    if(NOT TARGET NvSCI::NvSciEvent)
      add_library(NvSCI::NvSciEvent SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciEvent PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciEvent_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCIEVENT_SONAME}"
        INTERFACE_LINK_LIBRARIES "NvSCI::NvSciIpc;Threads::Threads"
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvSCI_INCLUDE_DIR}"
      )
    endif()
  endif()

  if(NvSCI_NvSciSync_LIBRARY)
    set(NvSCI_NvSciSync_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
    set(NvSCI_NvSciSync_LIBRARIES
      "${NvSCI_NvSciSync_LIBRARY}"
      "${NvSCI_NvSciBuf_LIBRARY}"
      "${NvSCI_NvSciCommon_LIBRARY}"
      "${NvSCI_NvSciIpc_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )
    mark_as_advanced(NvSCI_NvSciSync_LIBRARY)
    list(APPEND NvSCI_LIBRARIES "${NvSCI_NvSciSync_LIBRARY}")

    if(NOT TARGET NvSCI::NvSciSync)
      add_library(NvSCI::NvSciSync SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciSync PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciSync_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCISYNC_SONAME}"
        INTERFACE_LINK_LIBRARIES
          "NvSCI::NvSciBuf;NvSCI::NvSciCommon;NvSCI::NvSciIpc;Threads::Threads"
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvSCI_INCLUDE_DIR}"
      )
    endif()
  endif()

  if(NvSCI_NvSciStream_LIBRARY)
    set(NvSCI_NvSciStream_INCLUDE_DIRS "${NvSCI_INCLUDE_DIR}")
    set(NvSCI_NvSciStream_LIBRARIES
      "${NvSCI_NvSciStream_LIBRARY}"
      "${NvSCI_NvSciBuf_LIBRARY}"
      "${NvSCI_NvSciCommon_LIBRARY}"
      "${NvSCI_NvSciIpc_LIBRARY}"
      "${NvSCI_NvSciSync_LIBRARY}"
      ${CMAKE_THREAD_LIBS_INIT}
    )
    mark_as_advanced(NvSCI_NvSciStream_LIBRARY)
    list(APPEND NvSCI_LIBRARIES "${NvSCI_NvSciStream_LIBRARY}")

    if(NOT TARGET NvSCI::NvSciStream)
      add_library(NvSCI::NvSciStream SHARED IMPORTED)
      set_target_properties(NvSCI::NvSciStream PROPERTIES
        IMPORTED_LOCATION "${NvSCI_NvSciStream_LIBRARY}"
        IMPORTED_SONAME "${_NVSCI_NVSCISTREAM_SONAME}"
        INTERFACE_LINK_LIBRARIES
          "NvSCI::NvSciBuf;NvSCI::NvSciCommon;NvSCI::NvSciIpc;NvSCI::NvSciSync;Threads::Threads"
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvSCI_INCLUDE_DIR}"
      )
    endif()
  endif()

  list(APPEND NvSCI_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
endif()

unset(_NVSCI_NVSCIBUF_SONAME)
unset(_NVSCI_NVSCICOMMON_SONAME)
unset(_NVSCI_NVSCIEVENT_SONAME)
unset(_NVSCI_NVSCIIPC_SONAME)
unset(_NVSCI_NVSCISTREAM_SONAME)
unset(_NVSCI_NVSCISYNC_SONAME)
