cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(usv_simulation)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# Hello World
add_library(HelloWorld SHARED HelloWorld.cc)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})


# Push model
add_library(ModelPush SHARED ModelPush.cc
        utility.cc)
set_property(TARGET ModelPush PROPERTY CXX_STANDARD 17)
target_link_libraries(ModelPush
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
