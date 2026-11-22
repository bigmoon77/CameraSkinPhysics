#include "sphere_mesh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
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

        // Close the upper and lower boundary rings without introducing extra vertices.
        for (std::uint32_t segment = 1; segment + 1 < segment_count; ++segment)
        {
            mesh.indices.insert(mesh.indices.end(), {0, segment + 1, segment});

            const std::uint32_t bottom_base = (ring_count - 1) * segment_count;
            mesh.indices.insert(mesh.indices.end(), {
                bottom_base, bottom_base + segment, bottom_base + segment + 1
            });
        }

        build_surface_constraints(mesh);
        return mesh;
    }

    void build_surface_constraints(sphere_mesh& mesh,
        const float edge_compliance,
        const float area_compliance,
        const float bending_compliance)
    {
        if (mesh.indices.size() % 3u != 0u)
            throw std::invalid_argument("Surface index count must be divisible by three.");
        if (mesh.vertices.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())
            || mesh.indices.size() / 3u > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max() / 3u))
            throw std::invalid_argument("Surface topology exceeds 32-bit adjacency encoding limits.");

        struct edge_face
        {
            std::uint32_t face_index;
            std::uint32_t from;
            std::uint32_t to;
            std::uint32_t opposite;
        };

        struct edge_record
        {
            std::array<std::uint32_t, 2> key;
            edge_face face;
        };

        using edge_key = std::array<std::uint32_t, 2>;
        std::vector<edge_record> edge_records;
        edge_records.reserve(mesh.indices.size());
        std::vector<std::array<float, 3>> face_normals(mesh.indices.size() / 3u);
        mesh.edges.clear();
        mesh.edge_incidence_ranges.clear();
        mesh.incident_corrections.clear();
        mesh.triangle_areas.clear();
        mesh.triangle_incidence_ranges.clear();
        mesh.incident_triangle_corners.clear();
        mesh.bending.clear();
        mesh.topology = {};

        mesh.triangle_areas.reserve(mesh.indices.size() / 3u);
        std::vector<std::uint32_t> area_incidence_counts(mesh.vertices.size(), 0u);
        double signed_volume = 0.0;
        for (std::uint32_t face = 0; face < mesh.indices.size() / 3u; ++face)
        {
            const std::uint32_t a_index = mesh.indices[face * 3u];
            const std::uint32_t b_index = mesh.indices[face * 3u + 1u];
            const std::uint32_t c_index = mesh.indices[face * 3u + 2u];
            if (a_index >= mesh.vertices.size() || b_index >= mesh.vertices.size()
                || c_index >= mesh.vertices.size()
                || a_index == b_index || b_index == c_index || c_index == a_index)
                throw std::invalid_argument("Surface triangle contains invalid vertex indices.");

            const vertex& a = mesh.vertices[a_index];
            const vertex& b = mesh.vertices[b_index];
            const vertex& c = mesh.vertices[c_index];
            const float ab[3] = {b.position[0] - a.position[0], b.position[1] - a.position[1], b.position[2] - a.position[2]};
            const float ac[3] = {c.position[0] - a.position[0], c.position[1] - a.position[1], c.position[2] - a.position[2]};
            const float normal[3] = {
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0]
            };
            const float twice_area = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            if (!(twice_area > 1.0e-12f) || !std::isfinite(twice_area))
                throw std::invalid_argument("Surface contains a degenerate rest triangle.");

            face_normals[face] = {normal[0] / twice_area, normal[1] / twice_area, normal[2] / twice_area};
            triangle_area_constraint area{};
            area.indices[0] = a_index;
            area.indices[1] = b_index;
            area.indices[2] = c_index;
            area.rest_area = 0.5f * twice_area;
            area.compliance = area_compliance;
            mesh.triangle_areas.push_back(area);
            ++area_incidence_counts[a_index];
            ++area_incidence_counts[b_index];
            ++area_incidence_counts[c_index];

            signed_volume += (
                static_cast<double>(a.position[0]) * (static_cast<double>(b.position[1]) * c.position[2] - static_cast<double>(b.position[2]) * c.position[1])
                + static_cast<double>(a.position[1]) * (static_cast<double>(b.position[2]) * c.position[0] - static_cast<double>(b.position[0]) * c.position[2])
                + static_cast<double>(a.position[2]) * (static_cast<double>(b.position[0]) * c.position[1] - static_cast<double>(b.position[1]) * c.position[0])) / 6.0;

            const std::uint32_t ids[3] = {a_index, b_index, c_index};
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                const std::uint32_t from = ids[corner];
                const std::uint32_t to = ids[(corner + 1u) % 3u];
                const std::uint32_t opposite = ids[(corner + 2u) % 3u];
                const edge_key key = {std::min(from, to), std::max(from, to)};
                edge_records.push_back({key, {face, from, to, opposite}});
            }
        }

        std::sort(edge_records.begin(), edge_records.end(),
            [](const edge_record& a, const edge_record& b)
            {
                if (a.key != b.key) return a.key < b.key;
                return a.face.face_index < b.face.face_index;
            });

        std::vector<std::uint32_t> incidence_counts(mesh.vertices.size(), 0u);
        mesh.edges.reserve(edge_records.size() / 2u + 1u);
        mesh.bending.reserve(edge_records.size() / 2u + 1u);
        for (std::size_t record_begin = 0; record_begin < edge_records.size();)
        {
            std::size_t record_end = record_begin + 1u;
            while (record_end < edge_records.size()
                && edge_records[record_end].key == edge_records[record_begin].key)
                ++record_end;

            const edge_key& key = edge_records[record_begin].key;
            const std::size_t adjacent_count = record_end - record_begin;
            const vertex& a = mesh.vertices[key[0]];
            const vertex& b = mesh.vertices[key[1]];
            const float dx = a.position[0] - b.position[0];
            const float dy = a.position[1] - b.position[1];
            const float dz = a.position[2] - b.position[2];

            surface_edge edge{};
            edge.indices[0] = key[0];
            edge.indices[1] = key[1];
            edge.rest_length = std::sqrt(dx * dx + dy * dy + dz * dz);
            edge.compliance = edge_compliance;
            mesh.edges.push_back(edge);
            ++incidence_counts[key[0]];
            ++incidence_counts[key[1]];

            if (adjacent_count == 1u)
            {
                ++mesh.topology.boundary_edge_count;
                record_begin = record_end;
                continue;
            }
            if (adjacent_count != 2u)
            {
                ++mesh.topology.non_manifold_edge_count;
                record_begin = record_end;
                continue;
            }

            const edge_face& face0 = edge_records[record_begin].face;
            const edge_face& face1 = edge_records[record_begin + 1u].face;
            if (face0.from != face1.to || face0.to != face1.from)
                ++mesh.topology.orientation_conflict_count;

            const std::array<float, 3>& n0 = face_normals[face0.face_index];
            const std::array<float, 3>& n1 = face_normals[face1.face_index];
            const float edge_length = std::max(edge.rest_length, 1.0e-12f);
            const float edge_direction[3] = {dx / edge_length, dy / edge_length, dz / edge_length};
            const float normal_cross[3] = {
                n0[1] * n1[2] - n0[2] * n1[1],
                n0[2] * n1[0] - n0[0] * n1[2],
                n0[0] * n1[1] - n0[1] * n1[0]
            };
            const float sine = edge_direction[0] * normal_cross[0]
                + edge_direction[1] * normal_cross[1] + edge_direction[2] * normal_cross[2];
            const float cosine = std::clamp(n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2], -1.0f, 1.0f);

            bending_constraint bend{};
            bend.indices[0] = key[0];
            bend.indices[1] = key[1];
            bend.indices[2] = face0.opposite;
            bend.indices[3] = face1.opposite;
            bend.rest_angle = std::atan2(sine, cosine);
            bend.compliance = bending_compliance;
            mesh.bending.push_back(bend);
            record_begin = record_end;
        }

        mesh.edge_incidence_ranges.resize(mesh.vertices.size());
        std::uint32_t incidence_offset = 0u;
        for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index)
        {
            mesh.edge_incidence_ranges[vertex_index] = {incidence_offset, incidence_counts[vertex_index]};
            incidence_offset += incidence_counts[vertex_index];
        }

        mesh.incident_corrections.resize(incidence_offset);
        std::vector<std::uint32_t> write_offsets(mesh.vertices.size(), 0u);
        for (std::uint32_t edge_index = 0; edge_index < mesh.edges.size(); ++edge_index)
        {
            const surface_edge& edge = mesh.edges[edge_index];
            for (std::uint32_t endpoint = 0; endpoint < 2u; ++endpoint)
            {
                const std::uint32_t vertex_index = edge.indices[endpoint];
                const std::uint32_t destination =
                    mesh.edge_incidence_ranges[vertex_index].x + write_offsets[vertex_index]++;
                mesh.incident_corrections[destination] = edge_index * 2u + endpoint;
            }
        }

        mesh.triangle_incidence_ranges.resize(mesh.vertices.size());
        std::uint32_t triangle_incidence_offset = 0u;
        for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index)
        {
            mesh.triangle_incidence_ranges[vertex_index] = {
                triangle_incidence_offset, area_incidence_counts[vertex_index]};
            triangle_incidence_offset += area_incidence_counts[vertex_index];
        }

        mesh.incident_triangle_corners.resize(triangle_incidence_offset);
        std::vector<std::uint32_t> triangle_write_offsets(mesh.vertices.size(), 0u);
        for (std::uint32_t triangle_index = 0;
            triangle_index < static_cast<std::uint32_t>(mesh.triangle_areas.size()); ++triangle_index)
        {
            const triangle_area_constraint& area = mesh.triangle_areas[triangle_index];
            for (std::uint32_t corner = 0; corner < 3u; ++corner)
            {
                const std::uint32_t vertex_index = area.indices[corner];
                const std::uint32_t destination =
                    mesh.triangle_incidence_ranges[vertex_index].x + triangle_write_offsets[vertex_index]++;
                mesh.incident_triangle_corners[destination] = triangle_index * 3u + corner;
            }
        }

        mesh.topology.signed_rest_volume = static_cast<float>(signed_volume);
        mesh.topology.closed_oriented = mesh.topology.boundary_edge_count == 0u
            && mesh.topology.non_manifold_edge_count == 0u
            && mesh.topology.orientation_conflict_count == 0u
            && std::abs(mesh.topology.signed_rest_volume) > 1.0e-12f;
    }
}
