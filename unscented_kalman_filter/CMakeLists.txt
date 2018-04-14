project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/ukf.cpp src/main.cpp src/tools.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(UnscentedKF ${sources})

target_link_libraries(UnscentedKF z ssl uv uWS)


if(NOT without-test)
  enable_testing()
  set(utsources test/tools_spec.cpp src/tools.cpp)
  add_executable(UnitTest ${utsources})
  add_test(
    NAME UnitTest
    COMMAND $<TARGET_FILE:UnitTest>
  )
endif()
