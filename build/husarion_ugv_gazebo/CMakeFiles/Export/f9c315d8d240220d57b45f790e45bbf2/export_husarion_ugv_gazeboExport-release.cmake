#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "husarion_ugv_gazebo::estop_system" for configuration "Release"
set_property(TARGET husarion_ugv_gazebo::estop_system APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_gazebo::estop_system PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libestop_system.so"
  IMPORTED_SONAME_RELEASE "libestop_system.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_gazebo::estop_system )
list(APPEND _cmake_import_check_files_for_husarion_ugv_gazebo::estop_system "${_IMPORT_PREFIX}/lib/libestop_system.so" )

# Import target "husarion_ugv_gazebo::EStop" for configuration "Release"
set_property(TARGET husarion_ugv_gazebo::EStop APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_gazebo::EStop PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libEStop.so"
  IMPORTED_SONAME_RELEASE "libEStop.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_gazebo::EStop )
list(APPEND _cmake_import_check_files_for_husarion_ugv_gazebo::EStop "${_IMPORT_PREFIX}/lib/libEStop.so" )

# Import target "husarion_ugv_gazebo::LEDStrip" for configuration "Release"
set_property(TARGET husarion_ugv_gazebo::LEDStrip APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_gazebo::LEDStrip PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libLEDStrip.so"
  IMPORTED_SONAME_RELEASE "libLEDStrip.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_gazebo::LEDStrip )
list(APPEND _cmake_import_check_files_for_husarion_ugv_gazebo::LEDStrip "${_IMPORT_PREFIX}/lib/libLEDStrip.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
