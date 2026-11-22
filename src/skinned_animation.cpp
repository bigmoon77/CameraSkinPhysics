#include "skinned_animation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace csp
{
    namespace
    {
        constexpr std::uint32_t maximum_bone_count = 512u;
        constexpr std::uint32_t maximum_frame_count = 6000u;
        constexpr std::uint32_t maximum_vertex_count = 2000000u;

        bool finite_matrix(const skin_matrix& matrix)
        {
            for (const float value : matrix)
                if (!std::isfinite(value)) return false;
            return true;
        }
    }

    skeletal_animation load_skeletal_animation(
        const std::filesystem::path& path, std::uint32_t expected_vertex_count)
    {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("Failed to open skeletal animation file.");

        std::string magic;
        std::uint32_t version = 0;
        std::uint32_t vertex_count = 0;
        std::uint32_t bone_count = 0;
        std::uint32_t frame_count = 0;
        skeletal_animation animation{};
        if (!(file >> magic >> version >> vertex_count >> bone_count >> frame_count >> animation.frame_rate)
            || magic != "CSP_SKIN" || version != 1u)
            throw std::runtime_error("Invalid skeletal animation header.");
        if (vertex_count != expected_vertex_count || vertex_count > maximum_vertex_count
            || bone_count > maximum_bone_count || frame_count > maximum_frame_count
            || !std::isfinite(animation.frame_rate) || animation.frame_rate <= 0.0f)
            throw std::runtime_error("Skeletal animation dimensions are unsupported.");

        std::string section;
        if (!(file >> section) || section != "BONES")
            throw std::runtime_error("Skeletal animation is missing its bone section.");
        animation.bones.resize(bone_count);
        for (skin_bone& bone : animation.bones)
        {
            if (!(file >> std::quoted(bone.name) >> bone.parent_index))
                throw std::runtime_error("Invalid skeletal bone entry.");
            if (bone.parent_index < -1 || bone.parent_index >= static_cast<std::int32_t>(bone_count))
                throw std::runtime_error("Skeletal bone parent index is out of range.");
        }

        if (!(file >> section) || section != "WEIGHTS")
            throw std::runtime_error("Skeletal animation is missing its weight section.");
        animation.vertex_weights.resize(vertex_count);
        for (skin_vertex_weights& weights : animation.vertex_weights)
        {
            float total_weight = 0.0f;
            for (std::uint32_t influence = 0; influence < 4u; ++influence)
            {
                if (!(file >> weights.bone_indices[influence] >> weights.weights[influence]))
                    throw std::runtime_error("Invalid vertex skin weights.");
                if (!std::isfinite(weights.weights[influence]) || weights.weights[influence] < 0.0f)
                    throw std::runtime_error("Vertex skin weight is invalid.");
                if (weights.weights[influence] > 0.0f && weights.bone_indices[influence] >= bone_count)
                    throw std::runtime_error("Vertex skin bone index is out of range.");
                total_weight += weights.weights[influence];
            }
            if (bone_count > 0u && total_weight > 1.0e-6f)
            {
                for (float& weight : weights.weights) weight /= total_weight;
            }
            else
            {
                for (float& weight : weights.weights) weight = 0.0f;
            }
        }

        if (!(file >> section) || section != "FRAMES")
            throw std::runtime_error("Skeletal animation is missing its frame section.");
        animation.frames.resize(frame_count);
        for (std::vector<skin_matrix>& frame : animation.frames)
        {
            frame.resize(bone_count);
            for (skin_matrix& matrix : frame)
            {
                for (float& value : matrix)
                    if (!(file >> value)) throw std::runtime_error("Invalid bone animation matrix.");
                if (!finite_matrix(matrix))
                    throw std::runtime_error("Bone animation matrix contains a non-finite value.");
            }
        }
        if (!file) throw std::runtime_error("Failed while reading skeletal animation data.");
        return animation;
    }

    void sample_skin_matrices(const skeletal_animation& animation, float time_seconds,
        std::vector<skin_matrix>& output)
    {
        output.clear();
        if (animation.frames.empty() || animation.bones.empty()) return;

        const std::size_t frame_count = animation.frames.size();
        const float frame_position = std::fmod(std::max(0.0f, time_seconds) * animation.frame_rate,
            static_cast<float>(frame_count));
        const std::size_t frame_a = static_cast<std::size_t>(frame_position);
        const std::size_t frame_b = (frame_a + 1u) % frame_count;
        const float blend = frame_position - static_cast<float>(frame_a);
        output.resize(animation.bones.size());
        for (std::size_t bone_index = 0; bone_index < output.size(); ++bone_index)
        {
            for (std::size_t component = 0; component < 16u; ++component)
            {
                const float a = animation.frames[frame_a][bone_index][component];
                const float b = animation.frames[frame_b][bone_index][component];
                output[bone_index][component] = a + (b - a) * blend;
            }
        }
    }

    std::array<float, 3> skin_vertex_position(const std::array<float, 3>& position,
        const skin_vertex_weights& weights, const std::vector<skin_matrix>& matrices)
    {
        std::array<float, 3> result{};
        float total_weight = 0.0f;
        for (std::uint32_t influence = 0; influence < 4u; ++influence)
        {
            const float weight = weights.weights[influence];
            if (weight <= 0.0f) continue;
            const std::uint32_t bone_index = weights.bone_indices[influence];
            if (bone_index >= matrices.size()) continue;
            const skin_matrix& matrix = matrices[bone_index];
            result[0] += (matrix[0] * position[0] + matrix[1] * position[1]
                + matrix[2] * position[2] + matrix[3]) * weight;
            result[1] += (matrix[4] * position[0] + matrix[5] * position[1]
                + matrix[6] * position[2] + matrix[7]) * weight;
            result[2] += (matrix[8] * position[0] + matrix[9] * position[1]
                + matrix[10] * position[2] + matrix[11]) * weight;
            total_weight += weight;
        }
        if (total_weight <= 1.0e-6f) return position;
        const float inverse_weight = 1.0f / total_weight;
        for (float& component : result) component *= inverse_weight;
        return result;
    }
}
