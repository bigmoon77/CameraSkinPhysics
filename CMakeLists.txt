cmake_minimum_required(VERSION 3.24)
project(CameraSkinPhysics LANGUAGES CXX)
if(NOT WIN32)
    message(FATAL_ERROR "CameraSkinPhysics currently requires Windows and Direct3D 12.")
endif()
add_executable(CameraSkinPhysics WIN32
    src/main.cpp
    src/d3d12_renderer.cpp
    src/sphere_mesh.cpp
    src/benchmark_log.cpp
)
target_compile_features(CameraSkinPhysics PRIVATE cxx_std_20)
target_compile_definitions(CameraSkinPhysics PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(CameraSkinPhysics PRIVATE include)
target_link_libraries(CameraSkinPhysics PRIVATE d3d12 dxgi dxguid)
if(MSVC)
    target_compile_options(CameraSkinPhysics PRIVATE /W4 /permissive- /Zc:preprocessor /EHsc)
endif()
file(COPY shaders DESTINATION \${CMAKE_CURRENT_BINARY_DIR})
