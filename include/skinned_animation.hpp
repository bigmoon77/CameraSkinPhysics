#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace csp
{
    struct skin_vertex_weights
    {
        std::uint32_t bone_indices[4]{};
        float weights[4]{};
    };

    struct skin_bone
    {
        std::string name;
        std::int32_t parent_index = -1;
    };

    using skin_matrix = std::array<float, 16>;

    struct skeletal_animation
    {
        float frame_rate = 30.0f;
        std::vector<skin_bone> bones;
        std::vector<skin_vertex_weights> vertex_weights;
        std::vector<std::vector<skin_matrix>> frames;
    };

    skeletal_animation load_skeletal_animation(
        const std::filesystem::path& path, std::uint32_t expected_vertex_count);

    void sample_skin_matrices(const skeletal_animation& animation, float time_seconds,
        std::vector<skin_matrix>& output);

    std::array<float, 3> skin_vertex_position(const std::array<float, 3>& position,
        const skin_vertex_weights& weights, const std::vector<skin_matrix>& matrices);
}
