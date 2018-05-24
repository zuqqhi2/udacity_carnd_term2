project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/particle_filter.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(particle_filter ${sources})


target_link_libraries(particle_filter z ssl uv uWS)


if(NOT without-test)
  enable_testing()

  set(particle_filter_spec_sources test/particle_filter_spec.cpp src/particle_filter.cpp)
  add_executable(particle_filter_spec ${particle_filter_spec_sources})
  add_test(
    NAME particle_filter_spec
    COMMAND $<TARGET_FILE:particle_filter_spec>
  )
endif()
