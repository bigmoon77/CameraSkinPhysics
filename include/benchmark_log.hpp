#pragma once
#include <cstdint>
#include <filesystem>
#include <fstream>

namespace csp
{
    class benchmark_log
    {
    public:
        explicit benchmark_log(const std::filesystem::path& path);
        ~benchmark_log();
        benchmark_log(const benchmark_log&) = delete;
        benchmark_log& operator=(const benchmark_log&) = delete;

        void write_frame(std::uint64_t frame_index, double cpu_frame_ms,
            double gpu_simulation_ms, double camera_constraint_solve_ms,
            double camera_edge_solve_ms, double camera_area_solve_ms,
            double camera_bending_solve_ms, double camera_gather_ms, double gpu_depth_capture_ms,
            double gpu_collision_ms, double gpu_collision_compute_ms,
            double gpu_contact_apply_ms,
            double gpu_post_collision_skinning_body0_ms,
            double gpu_post_collision_skinning_body1_ms,
            double gpu_post_collision_skinning_ms,
            double gpu_render_ms,
            std::uint64_t simulation_particle_count,
            std::uint64_t tetrahedron_count,
            std::uint64_t solver_buffer_bytes,
            std::uint64_t resident_simulation_buffer_bytes,
            std::uint32_t solver_iterations, float edge_relaxation,
            float max_correction, float particle_radius, float pushout,
            float velocity_friction, float restitution, float contact_band,
            float shape_matching_strength, std::uint32_t capture_resolution);

    private:
        std::ofstream stream_;
    };
}
