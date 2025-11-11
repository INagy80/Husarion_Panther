#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "husarion_ugv_lights::animation_plugins" for configuration "Release"
set_property(TARGET husarion_ugv_lights::animation_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_lights::animation_plugins PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libanimation_plugins.so"
  IMPORTED_SONAME_RELEASE "libanimation_plugins.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_lights::animation_plugins )
list(APPEND _cmake_import_check_files_for_husarion_ugv_lights::animation_plugins "${_IMPORT_PREFIX}/lib/libanimation_plugins.so" )

# Import target "husarion_ugv_lights::lights_driver_node_component" for configuration "Release"
set_property(TARGET husarion_ugv_lights::lights_driver_node_component APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_lights::lights_driver_node_component PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblights_driver_node_component.so"
  IMPORTED_SONAME_RELEASE "liblights_driver_node_component.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_lights::lights_driver_node_component )
list(APPEND _cmake_import_check_files_for_husarion_ugv_lights::lights_driver_node_component "${_IMPORT_PREFIX}/lib/liblights_driver_node_component.so" )

# Import target "husarion_ugv_lights::lights_controller_node_component" for configuration "Release"
set_property(TARGET husarion_ugv_lights::lights_controller_node_component APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(husarion_ugv_lights::lights_controller_node_component PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblights_controller_node_component.so"
  IMPORTED_SONAME_RELEASE "liblights_controller_node_component.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_lights::lights_controller_node_component )
list(APPEND _cmake_import_check_files_for_husarion_ugv_lights::lights_controller_node_component "${_IMPORT_PREFIX}/lib/liblights_controller_node_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
