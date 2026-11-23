cmake_minimum_required(VERSION 3.24)
project(CameraSkinPhysics LANGUAGES CXX)

option(CSP_BUILD_TESTS "Build portable geometry and data validation tests" ON)
option(CSP_BUILD_D3D12_APP "Build the Direct3D 12 demo application" ON)

add_library(CameraSkinPhysicsCore
    src/sphere_mesh.cpp
    src/obj_mesh.cpp
    src/skinned_animation.cpp
    src/shape_matching.cpp
    src/tetrahedral_mesh.cpp
    src/benchmark_log.cpp
)
target_compile_features(CameraSkinPhysicsCore PUBLIC cxx_std_20)
target_include_directories(CameraSkinPhysicsCore PUBLIC include)

if(MSVC)
    target_compile_options(CameraSkinPhysicsCore PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(CameraSkinPhysicsCore PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CSP_BUILD_TESTS)
    include(CTest)
    add_executable(CameraSkinPhysicsTests tests/mesh_validation.cpp)
    target_link_libraries(CameraSkinPhysicsTests PRIVATE CameraSkinPhysicsCore)
    add_test(NAME CameraSkinPhysics.MeshValidation COMMAND CameraSkinPhysicsTests)

    add_executable(CameraSkinPhysicsObjTests tests/obj_mesh_validation.cpp)
    target_link_libraries(CameraSkinPhysicsObjTests PRIVATE CameraSkinPhysicsCore)
    add_test(NAME CameraSkinPhysics.ObjMeshValidation COMMAND CameraSkinPhysicsObjTests)

    add_executable(CameraSkinPhysicsBenchmarkLogTests tests/benchmark_log_validation.cpp)
    target_link_libraries(CameraSkinPhysicsBenchmarkLogTests PRIVATE CameraSkinPhysicsCore)
    add_test(NAME CameraSkinPhysics.BenchmarkLogValidation COMMAND CameraSkinPhysicsBenchmarkLogTests)

    add_executable(CameraSkinPhysicsSkinningTests tests/skinned_animation_validation.cpp)
    target_link_libraries(CameraSkinPhysicsSkinningTests PRIVATE CameraSkinPhysicsCore)
    add_test(NAME CameraSkinPhysics.SkeletalAnimationValidation COMMAND CameraSkinPhysicsSkinningTests)
endif()

if(WIN32 AND CSP_BUILD_D3D12_APP)
    find_program(DXC_EXECUTABLE NAMES dxc REQUIRED)
    set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
    file(MAKE_DIRECTORY "${SHADER_OUTPUT_DIR}")

    function(compile_hlsl source entry profile output_name)
        set(output "${SHADER_OUTPUT_DIR}/${output_name}.dxil")
        add_custom_command(
            OUTPUT "${output}"
            COMMAND "${DXC_EXECUTABLE}" -nologo -E "${entry}" -T "${profile}" -Ges -O3
                -Fo "${output}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${source}"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${source}"
            VERBATIM
        )
        set(SHADER_OUTPUTS ${SHADER_OUTPUTS} "${output}" PARENT_SCOPE)
    endfunction()

    compile_hlsl(integrate_particles.hlsl main cs_6_0 integrate_particles)
    compile_hlsl(clear_lambdas.hlsl main cs_6_0 clear_lambdas)
    compile_hlsl(xpbd_tetrahedra.hlsl main cs_6_0 xpbd_tetrahedra)
    compile_hlsl(xpbd_gather.hlsl main cs_6_0 xpbd_gather)
    compile_hlsl(skin_vertices.hlsl main cs_6_0 skin_vertices)
    compile_hlsl(camera_collision.hlsl main cs_6_0 camera_collision)
    compile_hlsl(camera_contact_smoothing.hlsl apply_corrections cs_6_0 camera_contact_apply)
    compile_hlsl(camera_surface_edges.hlsl main cs_6_0 camera_surface_edges)
    compile_hlsl(camera_surface_area.hlsl main cs_6_0 camera_surface_area)
    compile_hlsl(camera_surface_bending.hlsl main cs_6_0 camera_surface_bending)
    compile_hlsl(camera_surface_gather.hlsl main cs_6_0 camera_surface_gather)
    compile_hlsl(camera_surface_face_normals.hlsl main cs_6_0 camera_surface_face_normals)
    compile_hlsl(camera_surface_skin.hlsl main cs_6_0 camera_surface_skin)
    compile_hlsl(camera_surface_shape_matching.hlsl build_transforms cs_6_0 camera_surface_shape_build)
    compile_hlsl(camera_surface_shape_matching.hlsl apply_matching cs_6_0 camera_surface_shape_apply)
    compile_hlsl(capture_surface.hlsl VSMain vs_6_0 capture_surface_vs)
    compile_hlsl(capture_surface.hlsl PSMain ps_6_0 capture_surface_ps)
    compile_hlsl(render_sphere.hlsl VSMain vs_6_0 render_sphere_vs)
    compile_hlsl(render_sphere.hlsl PSMain ps_6_0 render_sphere_ps)
    add_custom_target(CameraSkinPhysicsShaders ALL DEPENDS ${SHADER_OUTPUTS})

    add_executable(CameraSkinPhysics WIN32
        src/main.cpp
        src/d3d12_renderer.cpp
    )
    add_dependencies(CameraSkinPhysics CameraSkinPhysicsShaders)
    target_compile_features(CameraSkinPhysics PRIVATE cxx_std_20)
    target_compile_definitions(CameraSkinPhysics PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_include_directories(CameraSkinPhysics PRIVATE include)
    target_link_libraries(CameraSkinPhysics PRIVATE CameraSkinPhysicsCore d3d12 dxgi dxguid comctl32 comdlg32 shell32)

    add_custom_command(TARGET CameraSkinPhysics POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${SHADER_OUTPUT_DIR}" "$<TARGET_FILE_DIR:CameraSkinPhysics>/shaders"
    )
elseif(CSP_BUILD_D3D12_APP)
    message(STATUS "Skipping the D3D12 application on non-Windows; portable core tests remain enabled.")
endif()
