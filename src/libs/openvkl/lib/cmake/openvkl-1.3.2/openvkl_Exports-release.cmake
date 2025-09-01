#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "openvkl::openvkl" for configuration "Release"
set_property(TARGET openvkl::openvkl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(openvkl::openvkl PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/openvkl.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/openvkl.dll"
  )

list(APPEND _cmake_import_check_targets openvkl::openvkl )
list(APPEND _cmake_import_check_files_for_openvkl::openvkl "${_IMPORT_PREFIX}/lib/openvkl.lib" "${_IMPORT_PREFIX}/bin/openvkl.dll" )

# Import target "openvkl::openvkl_module_cpu_device_4" for configuration "Release"
set_property(TARGET openvkl::openvkl_module_cpu_device_4 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(openvkl::openvkl_module_cpu_device_4 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_4.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "embree"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_4.dll"
  )

list(APPEND _cmake_import_check_targets openvkl::openvkl_module_cpu_device_4 )
list(APPEND _cmake_import_check_files_for_openvkl::openvkl_module_cpu_device_4 "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_4.lib" "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_4.dll" )

# Import target "openvkl::openvkl_module_cpu_device_8" for configuration "Release"
set_property(TARGET openvkl::openvkl_module_cpu_device_8 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(openvkl::openvkl_module_cpu_device_8 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_8.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "embree"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_8.dll"
  )

list(APPEND _cmake_import_check_targets openvkl::openvkl_module_cpu_device_8 )
list(APPEND _cmake_import_check_files_for_openvkl::openvkl_module_cpu_device_8 "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_8.lib" "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_8.dll" )

# Import target "openvkl::openvkl_module_cpu_device_16" for configuration "Release"
set_property(TARGET openvkl::openvkl_module_cpu_device_16 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(openvkl::openvkl_module_cpu_device_16 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_16.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "embree"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_16.dll"
  )

list(APPEND _cmake_import_check_targets openvkl::openvkl_module_cpu_device_16 )
list(APPEND _cmake_import_check_files_for_openvkl::openvkl_module_cpu_device_16 "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device_16.lib" "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device_16.dll" )

# Import target "openvkl::openvkl_module_cpu_device" for configuration "Release"
set_property(TARGET openvkl::openvkl_module_cpu_device APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(openvkl::openvkl_module_cpu_device PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device.dll"
  )

list(APPEND _cmake_import_check_targets openvkl::openvkl_module_cpu_device )
list(APPEND _cmake_import_check_files_for_openvkl::openvkl_module_cpu_device "${_IMPORT_PREFIX}/lib/openvkl_module_cpu_device.lib" "${_IMPORT_PREFIX}/bin/openvkl_module_cpu_device.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
