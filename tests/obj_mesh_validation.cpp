#include "obj_mesh.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }
}

int main()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "camera_skin_physics_obj_test.obj";
    try
    {
        {
            std::ofstream output(path);
            output << "v -2 -1 -1\n"
                << "v 2 -1 -1\n"
                << "v 2 1 -1\n"
                << "v -2 1 -1\n"
                << "v 0 0 1\n"
                << "f 1 2 3 4\n"
                << "f -5 -2 -1\n";
        }

        const csp::sphere_mesh mesh = csp::load_obj_mesh(path);
        require(mesh.vertices.size() == 5, "OBJ vertex count mismatch.");
        require(mesh.indices.size() == 9, "OBJ polygon triangulation failed.");
        require(mesh.edges.size() > 0, "OBJ edge topology was not generated.");
        require(mesh.triangle_areas.size() == 3, "OBJ triangle constraints were not generated.");
        require(mesh.edge_incidence_ranges.size() == mesh.vertices.size(),
            "OBJ edge incidence count mismatch.");

        float minimum_x = mesh.vertices.front().position[0];
        float maximum_x = minimum_x;
        for (const csp::vertex& vertex : mesh.vertices)
        {
            minimum_x = std::min(minimum_x, vertex.position[0]);
            maximum_x = std::max(maximum_x, vertex.position[0]);
            const float normal_length = std::sqrt(vertex.normal[0] * vertex.normal[0]
                + vertex.normal[1] * vertex.normal[1] + vertex.normal[2] * vertex.normal[2]);
            require(std::isfinite(normal_length), "OBJ normal is not finite.");
        }
        require(std::abs((maximum_x - minimum_x) - 2.0f) < 1.0e-5f,
            "OBJ normalization failed.");

        std::filesystem::remove(path);
        std::cout << "OBJ mesh validation passed.\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::filesystem::remove(path);
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
