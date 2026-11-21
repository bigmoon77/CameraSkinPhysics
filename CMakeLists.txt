cmake_minimum_required(VERSION 3.24)
project(CameraSkinPhysics LANGUAGES CXX)
if(NOT WIN32)
    message(FATAL_ERROR "CameraSkinPhysics currently requires Windows and Direct3D 12.")
endif()
find_program(DXC_EXECUTABLE NAMES dxc REQUIRED)
set(SHADER_OUTPUT_DIR "\${CMAKE_CURRENT_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "\${SHADER_OUTPUT_DIR}")
function(compile_hlsl source entry profile output_name)
    set(output "\${SHADER_OUTPUT_DIR}/\${output_name}.dxil")
    add_custom_command(OUTPUT "\${output}"
        COMMAND "\${DXC_EXECUTABLE}" -nologo -E "\${entry}" -T "\${profile}" -Ges -O3
            -Fo "\${output}" "\${CMAKE_CURRENT_SOURCE_DIR}/shaders/\${source}"
        DEPENDS "\${CMAKE_CURRENT_SOURCE_DIR}/shaders/\${source}" VERBATIM)
    set(SHADER_OUTPUTS \${SHADER_OUTPUTS} "\${output}" PARENT_SCOPE)
endfunction()
compile_hlsl(integrate_particles.hlsl main cs_6_0 integrate_particles)
compile_hlsl(xpbd_tetrahedra.hlsl main cs_6_0 xpbd_tetrahedra)
compile_hlsl(xpbd_gather.hlsl main cs_6_0 xpbd_gather)
compile_hlsl(camera_collision.hlsl main cs_6_0 camera_collision)
compile_hlsl(capture_surface.hlsl VSMain vs_6_0 capture_surface_vs)
compile_hlsl(capture_surface.hlsl PSMain ps_6_0 capture_surface_ps)
add_custom_target(CameraSkinPhysicsShaders ALL DEPENDS \${SHADER_OUTPUTS})
add_executable(CameraSkinPhysics WIN32
    src/main.cpp
    src/d3d12_renderer.cpp
    src/sphere_mesh.cpp
    src/tetrahedral_mesh.cpp
    src/benchmark_log.cpp
)
add_dependencies(CameraSkinPhysics CameraSkinPhysicsShaders)
target_compile_features(CameraSkinPhysics PRIVATE cxx_std_20)
target_compile_definitions(CameraSkinPhysics PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(CameraSkinPhysics PRIVATE include)
target_link_libraries(CameraSkinPhysics PRIVATE d3d12 dxgi dxguid)
if(MSVC)
    target_compile_options(CameraSkinPhysics PRIVATE /W4 /permissive- /Zc:preprocessor /EHsc)
endif()
