## Copyright 2019 Intel Corporation
## SPDX-License-Identifier: Apache-2.0


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was openvklConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

## Include openvkl targets ##

include("${CMAKE_CURRENT_LIST_DIR}/openvkl_Exports.cmake")

check_required_components("openvkl")

## openvkl ISA build configuration ##

set(OPENVKL_ISA_SSE4 ON)
set(OPENVKL_ISA_AVX ON)
set(OPENVKL_ISA_AVX2 ON)
set(OPENVKL_ISA_AVX512KNL OFF)
set(OPENVKL_ISA_AVX512SKX ON)
set(OPENVKL_ISA_NEON )
set(OPENVKL_ISA_NEON2X )

## Standard signal that the package was found ##

set(openvkl_FOUND TRUE)
