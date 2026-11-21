#include "sphere_mesh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace csp
{
    sphere_mesh make_sphere_mesh(const std::uint32_t ring_count, const std::uint32_t segment_count)
    {
        if (ring_count < 2 || segment_count < 3)
            throw std::invalid_argument("Sphere resolution is too small.");

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
                const float x = sin_phi * std::cos(theta);
                const float y = std::cos(phi);
                const float z = sin_phi * std::sin(theta);
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
                mesh.indices.insert(mesh.indices.end(), {a, b, c, b, d, c});
            }
        }

        std::vector<std::array<std::uint32_t, 2>> edge_pairs;
        edge_pairs.reserve(mesh.indices.size() / 2);
        for (std::size_t triangle = 0; triangle < mesh.indices.size(); triangle += 3)
        {
            const std::uint32_t triangle_indices[3] = {
                mesh.indices[triangle], mesh.indices[triangle + 1], mesh.indices[triangle + 2]
            };
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                std::uint32_t a = triangle_indices[corner];
                std::uint32_t b = triangle_indices[(corner + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_pairs.push_back({a, b});
            }
        }

        std::sort(edge_pairs.begin(), edge_pairs.end());
        edge_pairs.erase(std::unique(edge_pairs.begin(), edge_pairs.end()), edge_pairs.end());

        std::vector<std::uint32_t> incidence_counts(mesh.vertices.size(), 0);
        mesh.edges.reserve(edge_pairs.size());
        for (const std::array<std::uint32_t, 2>& pair : edge_pairs)
        {
            const vertex& a = mesh.vertices[pair[0]];
            const vertex& b = mesh.vertices[pair[1]];
            const float dx = a.position[0] - b.position[0];
            const float dy = a.position[1] - b.position[1];
            const float dz = a.position[2] - b.position[2];

            surface_edge edge{};
            edge.indices[0] = pair[0];
            edge.indices[1] = pair[1];
            edge.rest_length = std::sqrt(dx * dx + dy * dy + dz * dz);
            edge.compliance = 2.0e-6f;
            mesh.edges.push_back(edge);
            ++incidence_counts[pair[0]];
            ++incidence_counts[pair[1]];
        }

        mesh.edge_incidence_ranges.resize(mesh.vertices.size());
        std::uint32_t incidence_offset = 0;
        for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index)
        {
            mesh.edge_incidence_ranges[vertex_index] = {incidence_offset, incidence_counts[vertex_index]};
            incidence_offset += incidence_counts[vertex_index];
        }

        mesh.incident_edges.resize(incidence_offset);
        std::vector<std::uint32_t> write_offsets(mesh.vertices.size(), 0);
        for (std::uint32_t edge_index = 0; edge_index < mesh.edges.size(); ++edge_index)
        {
            const surface_edge& edge = mesh.edges[edge_index];
            for (const std::uint32_t vertex_index : edge.indices)
            {
                const std::uint32_t destination =
                    mesh.edge_incidence_ranges[vertex_index].x + write_offsets[vertex_index]++;
                mesh.incident_edges[destination] = edge_index;
            }
        }

        return mesh;
    }
}
