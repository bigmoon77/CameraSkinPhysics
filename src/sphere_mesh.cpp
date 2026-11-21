#include "sphere_mesh.hpp"
#include <cmath>
#include <stdexcept>
namespace csp
{
    sphere_mesh make_sphere_mesh(const std::uint32_t ring_count, const std::uint32_t segment_count)
    {
        if (ring_count < 2 || segment_count < 3) throw std::invalid_argument("Sphere resolution is too small.");
        sphere_mesh mesh{};
        mesh.vertices.reserve(static_cast<std::size_t>(ring_count) * segment_count);
        mesh.indices.reserve(static_cast<std::size_t>(ring_count - 1) * segment_count * 6);
        constexpr float pi = 3.14159265358979323846f;
        for (std::uint32_t ring = 0; ring < ring_count; ++ring)
        {
            const float phi = pi * (static_cast<float>(ring) + 0.5f) / static_cast<float>(ring_count);
            const float sin_phi = std::sin(phi);
            for (std::uint32_t segment = 0; segment < segment_count; ++segment)
            {
                const float theta = 2.0f * pi * static_cast<float>(segment) / static_cast<float>(segment_count);
                const float x = sin_phi * std::cos(theta), y = std::cos(phi), z = sin_phi * std::sin(theta);
                vertex value{};
                value.position[0] = value.normal[0] = x;
                value.position[1] = value.normal[1] = y;
                value.position[2] = value.normal[2] = z;
                mesh.vertices.push_back(value);
            }
        }
        for (std::uint32_t ring = 0; ring + 1 < ring_count; ++ring)
        {
            for (std::uint32_t segment = 0; segment < segment_count; ++segment)
            {
                const std::uint32_t next = (segment + 1) % segment_count;
                const std::uint32_t a = ring * segment_count + segment;
                const std::uint32_t b = ring * segment_count + next;
                const std::uint32_t c = (ring + 1) * segment_count + segment;
                const std::uint32_t d = (ring + 1) * segment_count + next;
                mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
            }
        }
        return mesh;
    }
}
