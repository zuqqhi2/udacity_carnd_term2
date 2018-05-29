project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)

if(NOT without-test)
  enable_testing()

  set(pid_spec_sources test/pid_spec.cpp src/PID.cpp)
  add_executable(pid_spec ${pid_spec_sources})
  add_test(
    NAME pid_spec
    COMMAND $<TARGET_FILE:pid_spec>
  )
endif()
