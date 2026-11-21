#include "tetrahedral_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <stdexcept>

namespace csp
{
    namespace
    {
        struct float3
        {
            float x;
            float y;
            float z;
        };

        float3 subtract(const float3& a, const float3& b)
        {
            return {a.x - b.x, a.y - b.y, a.z - b.z};
        }

        float3 cross(const float3& a, const float3& b)
        {
            return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        }

        float dot(const float3& a, const float3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        float signed_volume(const float3& a, const float3& b, const float3& c, const float3& d)
        {
            return dot(cross(subtract(b, a), subtract(c, a)), subtract(d, a)) / 6.0f;
        }

        bool calculate_barycentric(const float3& p0, const float3& p1, const float3& p2,
            const float3& p3, const float3& point, float weights[4])
        {
            const float3 e1 = subtract(p1, p0);
            const float3 e2 = subtract(p2, p0);
            const float3 e3 = subtract(p3, p0);
            const float3 relative = subtract(point, p0);
            const float determinant = dot(e1, cross(e2, e3));
            if (std::abs(determinant) < 1.0e-10f) return false;

            weights[1] = dot(relative, cross(e2, e3)) / determinant;
            weights[2] = dot(e1, cross(relative, e3)) / determinant;
            weights[3] = dot(e1, cross(e2, relative)) / determinant;
            weights[0] = 1.0f - weights[1] - weights[2] - weights[3];
            return true;
        }

        float3 get_position(const std::vector<simulation_particle>& particles, const std::uint32_t index)
        {
            const simulation_particle& particle = particles[index];
            return {particle.position[0], particle.position[1], particle.position[2]};
        }
    }

    tetrahedral_mesh make_tetrahedral_mesh(const sphere_mesh& surface, const std::uint32_t grid_resolution)
    {
        if (grid_resolution < 4) throw std::invalid_argument("Tetrahedral grid resolution must be at least 4.");

        tetrahedral_mesh result{};
        const std::size_t grid_count = static_cast<std::size_t>(grid_resolution) * grid_resolution * grid_resolution;
        const std::uint32_t cell_resolution = grid_resolution - 1;
        const std::size_t cell_count = static_cast<std::size_t>(cell_resolution) * cell_resolution * cell_resolution;
        std::vector<std::int32_t> grid_to_particle(grid_count, -1);
        std::vector<std::array<std::int32_t, 6>> cell_to_tetrahedra(cell_count);
        for (auto& cell : cell_to_tetrahedra) cell.fill(-1);

        const auto grid_index = [grid_resolution](std::uint32_t x, std::uint32_t y, std::uint32_t z) -> std::size_t
        {
            return (static_cast<std::size_t>(z) * grid_resolution + y) * grid_resolution + x;
        };
        const auto cell_index = [cell_resolution](std::uint32_t x, std::uint32_t y, std::uint32_t z) -> std::size_t
        {
            return (static_cast<std::size_t>(z) * cell_resolution + y) * cell_resolution + x;
        };
        const auto coordinate = [grid_resolution](std::uint32_t index) -> float
        {
            return -1.0f + 2.0f * static_cast<float>(index) / static_cast<float>(grid_resolution - 1);
        };

        for (std::uint32_t z = 0; z < grid_resolution; ++z)
        {
            for (std::uint32_t y = 0; y < grid_resolution; ++y)
            {
                for (std::uint32_t x = 0; x < grid_resolution; ++x)
                {
                    const float px = coordinate(x);
                    const float py = coordinate(y);
                    const float pz = coordinate(z);
                    if (px * px + py * py + pz * pz > 1.0f) continue;

                    simulation_particle particle{};
                    particle.position[0] = px;
                    particle.position[1] = py;
                    particle.position[2] = pz;
                    particle.position[3] = 1.0f;
                    result.particles.push_back(particle);
                    result.inverse_masses.push_back(1.0f);
                    grid_to_particle[grid_index(x, y, z)] = static_cast<std::int32_t>(result.particles.size() - 1);
                }
            }
        }

        constexpr std::uint32_t local_tetrahedra[6][4] =
        {
            {0, 1, 3, 7}, {0, 3, 2, 7}, {0, 2, 6, 7},
            {0, 6, 4, 7}, {0, 4, 5, 7}, {0, 5, 1, 7}
        };
        constexpr std::uint32_t corner_offsets[8][3] =
        {
            {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0},
            {0,0,1}, {1,0,1}, {0,1,1}, {1,1,1}
        };

        for (std::uint32_t z = 0; z + 1 < grid_resolution; ++z)
        {
            for (std::uint32_t y = 0; y + 1 < grid_resolution; ++y)
            {
                for (std::uint32_t x = 0; x + 1 < grid_resolution; ++x)
                {
                    std::uint32_t corners[8]{};
                    bool complete_cell = true;
                    for (std::uint32_t corner = 0; corner < 8; ++corner)
                    {
                        const std::uint32_t gx = x + corner_offsets[corner][0];
                        const std::uint32_t gy = y + corner_offsets[corner][1];
                        const std::uint32_t gz = z + corner_offsets[corner][2];
                        const std::int32_t particle_index = grid_to_particle[grid_index(gx, gy, gz)];
                        if (particle_index < 0)
                        {
                            complete_cell = false;
                            break;
                        }
                        corners[corner] = static_cast<std::uint32_t>(particle_index);
                    }
                    if (!complete_cell) continue;

                    const std::size_t current_cell_index = cell_index(x, y, z);
                    for (std::uint32_t local_index = 0; local_index < 6; ++local_index)
                    {
                        const std::uint32_t* local_indices = local_tetrahedra[local_index];
                        tetrahedron tet{};
                        for (std::uint32_t i = 0; i < 4; ++i) tet.indices[i] = corners[local_indices[i]];

                        float volume = signed_volume(get_position(result.particles, tet.indices[0]),
                            get_position(result.particles, tet.indices[1]),
                            get_position(result.particles, tet.indices[2]),
                            get_position(result.particles, tet.indices[3]));
                        if (std::abs(volume) < 1.0e-9f) continue;
                        if (volume < 0.0f)
                        {
                            std::swap(tet.indices[1], tet.indices[2]);
                            volume = -volume;
                        }
                        tet.rest_volume = volume;
                        tet.compliance = 1.0e-7f;
                        tet.lambda = 0.0f;
                        cell_to_tetrahedra[current_cell_index][local_index] = static_cast<std::int32_t>(result.tetrahedra.size());
                        result.tetrahedra.push_back(tet);
                        result.tetrahedron_indices.push_back({tet.indices[0], tet.indices[1], tet.indices[2], tet.indices[3]});
                    }
                }
            }
        }

        std::vector<std::uint32_t> incidence_counts(result.particles.size(), 0);
        for (const tetrahedron& tet : result.tetrahedra)
        {
            for (const std::uint32_t particle_index : tet.indices) ++incidence_counts[particle_index];
        }

        result.incidence_ranges.resize(result.particles.size());
        std::uint32_t incidence_offset = 0;
        for (std::size_t particle_index = 0; particle_index < result.particles.size(); ++particle_index)
        {
            result.incidence_ranges[particle_index] = {incidence_offset, incidence_counts[particle_index]};
            incidence_offset += incidence_counts[particle_index];
        }

        result.incident_tetrahedra.resize(incidence_offset);
        std::vector<std::uint32_t> write_offsets(result.particles.size(), 0);
        for (std::uint32_t tet_index = 0; tet_index < result.tetrahedra.size(); ++tet_index)
        {
            for (const std::uint32_t particle_index : result.tetrahedra[tet_index].indices)
            {
                const std::uint32_t destination = result.incidence_ranges[particle_index].x + write_offsets[particle_index]++;
                result.incident_tetrahedra[destination] = tet_index;
            }
        }

        result.surface_mappings.reserve(surface.vertices.size());
        const float grid_scale = static_cast<float>(grid_resolution - 1) * 0.5f;
        constexpr float skin_inset = 0.88f;
        for (const vertex& surface_vertex : surface.vertices)
        {
            const float3 surface_position{surface_vertex.position[0], surface_vertex.position[1], surface_vertex.position[2]};
            const float3 target_position{surface_position.x * skin_inset,
                surface_position.y * skin_inset, surface_position.z * skin_inset};

            const auto to_cell = [grid_scale, cell_resolution](const float value) -> std::int32_t
            {
                const float scaled = (value + 1.0f) * grid_scale;
                const std::int32_t index = static_cast<std::int32_t>(std::floor(scaled));
                return std::clamp(index, 0, static_cast<std::int32_t>(cell_resolution) - 1);
            };
            const std::int32_t center_x = to_cell(target_position.x);
            const std::int32_t center_y = to_cell(target_position.y);
            const std::int32_t center_z = to_cell(target_position.z);

            surface_mapping mapping{};
            bool found = false;
            float best_distance = std::numeric_limits<float>::max();
            surface_mapping best_mapping{};

            for (std::int32_t radius = 0; radius <= 3 && !found; ++radius)
            {
                for (std::int32_t dz = -radius; dz <= radius; ++dz)
                {
                    for (std::int32_t dy = -radius; dy <= radius; ++dy)
                    {
                        for (std::int32_t dx = -radius; dx <= radius; ++dx)
                        {
                            if (radius > 0 && std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) continue;
                            const std::int32_t cx = center_x + dx;
                            const std::int32_t cy = center_y + dy;
                            const std::int32_t cz = center_z + dz;
                            if (cx < 0 || cy < 0 || cz < 0
                                || cx >= static_cast<std::int32_t>(cell_resolution)
                                || cy >= static_cast<std::int32_t>(cell_resolution)
                                || cz >= static_cast<std::int32_t>(cell_resolution)) continue;

                            const auto& candidates = cell_to_tetrahedra[cell_index(
                                static_cast<std::uint32_t>(cx), static_cast<std::uint32_t>(cy), static_cast<std::uint32_t>(cz))];
                            for (const std::int32_t candidate : candidates)
                            {
                                if (candidate < 0) continue;
                                const tetrahedron& tet = result.tetrahedra[static_cast<std::size_t>(candidate)];
                                const float3 p0 = get_position(result.particles, tet.indices[0]);
                                const float3 p1 = get_position(result.particles, tet.indices[1]);
                                const float3 p2 = get_position(result.particles, tet.indices[2]);
                                const float3 p3 = get_position(result.particles, tet.indices[3]);
                                float weights[4]{};
                                if (!calculate_barycentric(p0, p1, p2, p3, target_position, weights)) continue;

                                constexpr float barycentric_tolerance = 1.0e-4f;
                                if (weights[0] < -barycentric_tolerance || weights[1] < -barycentric_tolerance
                                    || weights[2] < -barycentric_tolerance || weights[3] < -barycentric_tolerance
                                    || weights[0] > 1.0f + barycentric_tolerance || weights[1] > 1.0f + barycentric_tolerance
                                    || weights[2] > 1.0f + barycentric_tolerance || weights[3] > 1.0f + barycentric_tolerance) continue;

                                const float3 center{
                                    (p0.x + p1.x + p2.x + p3.x) * 0.25f,
                                    (p0.y + p1.y + p2.y + p3.y) * 0.25f,
                                    (p0.z + p1.z + p2.z + p3.z) * 0.25f
                                };
                                const float3 delta = subtract(center, target_position);
                                const float distance = dot(delta, delta);
                                if (distance < best_distance)
                                {
                                    best_distance = distance;
                                    for (std::uint32_t i = 0; i < 4; ++i)
                                    {
                                        best_mapping.indices[i] = tet.indices[i];
                                        best_mapping.weights[i] = weights[i];
                                    }
                                }
                            }
                        }
                    }
                }
                if (best_distance < std::numeric_limits<float>::max()) found = true;
            }

            if (!found)
            {
                float nearest_distance = std::numeric_limits<float>::max();
                for (std::size_t tet_index = 0; tet_index < result.tetrahedra.size(); ++tet_index)
                {
                    const tetrahedron& tet = result.tetrahedra[tet_index];
                    const float3 p0 = get_position(result.particles, tet.indices[0]);
                    const float3 p1 = get_position(result.particles, tet.indices[1]);
                    const float3 p2 = get_position(result.particles, tet.indices[2]);
                    const float3 p3 = get_position(result.particles, tet.indices[3]);
                    const float3 center{
                        (p0.x + p1.x + p2.x + p3.x) * 0.25f,
                        (p0.y + p1.y + p2.y + p3.y) * 0.25f,
                        (p0.z + p1.z + p2.z + p3.z) * 0.25f
                    };
                    const float3 delta = subtract(center, target_position);
                    const float distance = dot(delta, delta);
                    if (distance >= nearest_distance) continue;

                    float weights[4]{};
                    if (!calculate_barycentric(p0, p1, p2, p3, target_position, weights)) continue;
                    float weight_sum = 0.0f;
                    for (float& weight : weights)
                    {
                        weight = std::max(0.0f, weight);
                        weight_sum += weight;
                    }
                    if (weight_sum <= 1.0e-8f)
                    {
                        const float distances[4] = {
                            dot(subtract(p0, target_position), subtract(p0, target_position)),
                            dot(subtract(p1, target_position), subtract(p1, target_position)),
                            dot(subtract(p2, target_position), subtract(p2, target_position)),
                            dot(subtract(p3, target_position), subtract(p3, target_position))
                        };
                        const std::uint32_t closest = static_cast<std::uint32_t>(
                            std::min_element(std::begin(distances), std::end(distances)) - std::begin(distances));
                        for (float& weight : weights) weight = 0.0f;
                        weights[closest] = 1.0f;
                        weight_sum = 1.0f;
                    }
                    for (std::uint32_t i = 0; i < 4; ++i)
                    {
                        best_mapping.indices[i] = tet.indices[i];
                        best_mapping.weights[i] = weights[i] / weight_sum;
                    }
                    nearest_distance = distance;
                }
                if (nearest_distance == std::numeric_limits<float>::max())
                    throw std::runtime_error("Failed to find a tetrahedron for a render vertex.");
                found = true;
            }

            const float3 interpolated{
                best_mapping.weights[0] * result.particles[best_mapping.indices[0]].position[0]
                    + best_mapping.weights[1] * result.particles[best_mapping.indices[1]].position[0]
                    + best_mapping.weights[2] * result.particles[best_mapping.indices[2]].position[0]
                    + best_mapping.weights[3] * result.particles[best_mapping.indices[3]].position[0],
                best_mapping.weights[0] * result.particles[best_mapping.indices[0]].position[1]
                    + best_mapping.weights[1] * result.particles[best_mapping.indices[1]].position[1]
                    + best_mapping.weights[2] * result.particles[best_mapping.indices[2]].position[1]
                    + best_mapping.weights[3] * result.particles[best_mapping.indices[3]].position[1],
                best_mapping.weights[0] * result.particles[best_mapping.indices[0]].position[2]
                    + best_mapping.weights[1] * result.particles[best_mapping.indices[1]].position[2]
                    + best_mapping.weights[2] * result.particles[best_mapping.indices[2]].position[2]
                    + best_mapping.weights[3] * result.particles[best_mapping.indices[3]].position[2]
            };
            best_mapping.rest_offset[0] = surface_position.x - interpolated.x;
            best_mapping.rest_offset[1] = surface_position.y - interpolated.y;
            best_mapping.rest_offset[2] = surface_position.z - interpolated.z;
            best_mapping.rest_offset[3] = 0.0f;
            mapping = best_mapping;
            result.surface_mappings.push_back(mapping);
        }

        return result;
    }
}
