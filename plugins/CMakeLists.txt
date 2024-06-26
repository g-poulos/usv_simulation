cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(usv_simulation)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})


# Water Current
add_library(WaterCurrent SHARED WaterCurrent.cc utility.cc)
set_property(TARGET WaterCurrent PROPERTY CXX_STANDARD 17)
target_link_libraries(WaterCurrent
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Wind
add_library(Wind SHARED Wind.cc utility.cc)
set_property(TARGET Wind PROPERTY CXX_STANDARD 17)
target_link_libraries(Wind
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Added Mass
add_library(AddedMass SHARED AddedMass.cc)
set_property(TARGET AddedMass PROPERTY CXX_STANDARD 17)
target_link_libraries(AddedMass
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Model State Publisher
add_library(ModelStatePublisher SHARED ModelStatePublisher.cc)
set_property(TARGET ModelStatePublisher PROPERTY CXX_STANDARD 17)
target_link_libraries(ModelStatePublisher
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})