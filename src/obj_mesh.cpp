#include "obj_mesh.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace csp
{
    sphere_mesh load_obj_mesh(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("Failed to open OBJ file.");
        std::vector<std::array<float, 3>> positions;
        std::vector<std::uint32_t> indices;
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream stream(line);
            std::string token;
            stream >> token;
            if (token == "v")
            {
                std::array<float, 3> value{};
                if (!(stream >> value[0] >> value[1] >> value[2]))
                    throw std::runtime_error("Invalid OBJ vertex.");
                positions.push_back(value);
            }
            else if (token == "f")
            {
                std::vector<std::uint32_t> face;
                std::string item;
                while (stream >> item)
                {
                    const std::size_t slash = item.find('/');
                    const long parsed = std::stol(item.substr(0, slash));
                    const long resolved = parsed > 0 ? parsed - 1
                        : static_cast<long>(positions.size()) + parsed;
                    if (resolved < 0 || resolved >= static_cast<long>(positions.size()))
                        throw std::runtime_error("OBJ face index is out of range.");
                    face.push_back(static_cast<std::uint32_t>(resolved));
                }
                for (std::size_t corner = 1; corner + 1 < face.size(); ++corner)
                    indices.insert(indices.end(), {face[0], face[corner], face[corner + 1]});
            }
        }
        if (positions.size() < 3 || indices.empty())
            throw std::runtime_error("OBJ contains no triangle mesh.");
        if (positions.size() > 2000000 || indices.size() > 12000000)
            throw std::runtime_error("OBJ exceeds the supported import size.");

        std::array<float, 3> minimum = positions.front();
        std::array<float, 3> maximum = positions.front();
        for (const auto& p : positions)
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                minimum[axis] = std::min(minimum[axis], p[axis]);
                maximum[axis] = std::max(maximum[axis], p[axis]);
            }
        const std::array<float, 3> center = {
            (minimum[0] + maximum[0]) * 0.5f,
            (minimum[1] + maximum[1]) * 0.5f,
            (minimum[2] + maximum[2]) * 0.5f
        };
        const float extent = std::max({maximum[0] - minimum[0],
            maximum[1] - minimum[1], maximum[2] - minimum[2]});
        if (!(extent > 1.0e-6f) || !std::isfinite(extent))
            throw std::runtime_error("OBJ has invalid bounds.");

        sphere_mesh mesh{};
        mesh.vertices.resize(positions.size());
        const float scale = 2.0f / extent;
        for (std::size_t i = 0; i < positions.size(); ++i)
            for (std::size_t axis = 0; axis < 3; ++axis)
                mesh.vertices[i].position[axis] = (positions[i][axis] - center[axis]) * scale;
        mesh.indices = std::move(indices);

        for (std::size_t face = 0; face < mesh.indices.size(); face += 3)
        {
            const vertex& a = mesh.vertices[mesh.indices[face]];
            const vertex& b = mesh.vertices[mesh.indices[face + 1]];
            const vertex& c = mesh.vertices[mesh.indices[face + 2]];
            const float ab[3] = {b.position[0] - a.position[0], b.position[1] - a.position[1], b.position[2] - a.position[2]};
            const float ac[3] = {c.position[0] - a.position[0], c.position[1] - a.position[1], c.position[2] - a.position[2]};
            const float n[3] = {ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]};
            for (std::size_t corner = 0; corner < 3; ++corner)
                for (std::size_t axis = 0; axis < 3; ++axis)
                    mesh.vertices[mesh.indices[face + corner]].normal[axis] += n[axis];
        }
        for (vertex& v : mesh.vertices)
        {
            const float length = std::sqrt(v.normal[0] * v.normal[0]
                + v.normal[1] * v.normal[1] + v.normal[2] * v.normal[2]);
            if (length > 1.0e-12f)
                for (float& component : v.normal) component /= length;
            else v.normal[1] = 1.0f;
        }
        build_surface_constraints(mesh);
        return mesh;
    }
}
