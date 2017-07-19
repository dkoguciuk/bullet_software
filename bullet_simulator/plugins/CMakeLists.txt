# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# Depend on boost
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Set plugins to build
set(plugins
  hello_world
  model_push
)

# Plugins loop
foreach (src ${plugins})
  add_library(${src} SHARED ${PROJECT_SOURCE_DIR}/plugins/${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#  install(TARGETS ${src} DESTINATION ${PROJECT_SOURCE_DIR}/plugins)
#  install(FILES ${src}.cc DESTINATION ${PROJECT_SOURCE_DIR}/plugins COMPONENT headers)
#  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})
