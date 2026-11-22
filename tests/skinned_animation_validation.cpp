#include "skinned_animation.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    bool near(float a, float b, float tolerance = 1.0e-5f)
    {
        return std::abs(a - b) <= tolerance;
    }
}

int main()
{
    try
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() / "csp_skin_validation.cspanim";
        {
            std::ofstream file(path);
            file << "CSP_SKIN 1 1 2 2 2\n";
            file << "BONES\n";
            file << "\"root\" -1\n";
            file << "\"child\" 0\n";
            file << "WEIGHTS\n";
            file << "0 0.25 1 0.75 0 0 0 0\n";
            file << "FRAMES\n";
            file << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n";
            file << "1 0 0 2 0 1 0 0 0 0 1 0 0 0 0 1\n";
            file << "1 0 0 2 0 1 0 0 0 0 1 0 0 0 0 1\n";
            file << "1 0 0 4 0 1 0 0 0 0 1 0 0 0 0 1\n";
        }

        const csp::skeletal_animation animation = csp::load_skeletal_animation(path, 1u);
        require(animation.bones.size() == 2u, "Bone count mismatch.");
        require(animation.frames.size() == 2u, "Animation frame count mismatch.");
        require(near(animation.vertex_weights[0].weights[0], 0.25f), "Weight normalization failed.");
        require(near(animation.vertex_weights[0].weights[1], 0.75f), "Weight normalization failed.");

        std::vector<csp::skin_matrix> matrices;
        csp::sample_skin_matrices(animation, 0.25f, matrices);
        require(matrices.size() == 2u, "Sampled matrix count mismatch.");
        require(near(matrices[0][3], 0.5f), "Frame interpolation failed.");
        require(near(matrices[1][3], 2.5f), "Frame interpolation failed.");

        const std::array<float, 3> position{1.0f, 0.0f, 0.0f};
        const std::array<float, 3> skinned =
            csp::skin_vertex_position(position, animation.vertex_weights[0], matrices);
        require(near(skinned[0], 2.0f), "Weighted skinning result mismatch.");
        require(near(skinned[1], 0.0f) && near(skinned[2], 0.0f), "Unexpected skinning axis change.");

        std::filesystem::remove(path);
        std::cout << "Skeletal animation validation passed.\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
