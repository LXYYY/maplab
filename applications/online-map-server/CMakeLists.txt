cmake_minimum_required(VERSION 2.8.3)
project(online_map_server)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# ##############################################################################
# LIBRARIES #
# ##############################################################################
cs_add_library(${PROJECT_NAME}_lib src/online-map-server.cc)

#
# APP #
#
cs_add_executable(online_map_server app/online-map-server-app.cc)
target_link_libraries(online_map_server ${PROJECT_NAME}_lib)

# ##############################################################################
# EXPORT ##
# ##############################################################################
cs_install()
cs_export()
