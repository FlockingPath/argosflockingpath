# include_directories(${ARGOS_INCLUDE_DIRS})

add_library(footbot_flocking SHARED footbot_flocking.h footbot_flocking.cpp)

# # Specify the path to the ARGoS plugins
# link_directories(/usr/local/lib/argos3 /usr/lib/argos3)

target_link_libraries(footbot_flocking
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
  # argos3plugin_simulator_media
  # argos3plugin_simulator_range_and_bearing)
