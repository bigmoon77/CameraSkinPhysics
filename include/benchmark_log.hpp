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
        void write_frame(std::uint64_t frame_index, double cpu_frame_ms, double gpu_simulation_ms,
            double gpu_depth_capture_ms, double gpu_collision_ms, double gpu_render_ms,
            std::uint64_t simulation_particle_count, std::uint64_t tetrahedron_count);
    private:
        std::ofstream stream_;
    };
}
