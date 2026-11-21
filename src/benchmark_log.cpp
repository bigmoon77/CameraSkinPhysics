#include "benchmark_log.hpp"
#include <stdexcept>
namespace csp
{
    benchmark_log::benchmark_log(const std::filesystem::path& path)
    {
        if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
        const bool needs_header = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
        stream_.open(path, std::ios::out | std::ios::app);
        if (!stream_) throw std::runtime_error("Failed to open benchmark CSV.");
        if (needs_header) stream_ << "frame,cpu_frame_ms,gpu_simulation_ms,gpu_depth_capture_ms,gpu_collision_ms,gpu_render_ms,particle_count,tetrahedron_count\n";
    }
    benchmark_log::~benchmark_log() { stream_.flush(); }
    void benchmark_log::write_frame(std::uint64_t frame_index, double cpu_frame_ms, double gpu_simulation_ms,
        double gpu_depth_capture_ms, double gpu_collision_ms, double gpu_render_ms,
        std::uint64_t simulation_particle_count, std::uint64_t tetrahedron_count)
    {
        stream_ << frame_index << ',' << cpu_frame_ms << ',' << gpu_simulation_ms << ','
            << gpu_depth_capture_ms << ',' << gpu_collision_ms << ',' << gpu_render_ms << ','
            << simulation_particle_count << ',' << tetrahedron_count << '\n';
    }
}
