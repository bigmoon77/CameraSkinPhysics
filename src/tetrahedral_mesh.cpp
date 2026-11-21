#include "tetrahedral_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
    }

    tetrahedral_mesh make_tetrahedral_mesh(const sphere_mesh& surface, const std::uint32_t grid_resolution)
    {
        if (grid_resolution < 4) throw std::invalid_argument("Tetrahedral grid resolution must be at least 4.");

        tetrahedral_mesh result{};
        const std::size_t grid_count = static_cast<std::size_t>(grid_resolution) * grid_resolution * grid_resolution;
        std::vector<std::int32_t> grid_to_particle(grid_count, -1);

        const auto grid_index = [grid_resolution](std::uint32_t x, std::uint32_t y, std::uint32_t z) -> std::size_t
        {
            return (static_cast<std::size_t>(z) * grid_resolution + y) * grid_resolution + x;
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

                    for (const auto& local_indices : local_tetrahedra)
                    {
                        tetrahedron tet{};
                        for (std::uint32_t i = 0; i < 4; ++i) tet.indices[i] = corners[local_indices[i]];

                        const auto get_position = [&result](std::uint32_t index) -> float3
                        {
                            const simulation_particle& p = result.particles[index];
                            return {p.position[0], p.position[1], p.position[2]};
                        };
                        float volume = signed_volume(get_position(tet.indices[0]), get_position(tet.indices[1]),
                            get_position(tet.indices[2]), get_position(tet.indices[3]));
                        if (std::abs(volume) < 1.0e-9f) continue;
                        if (volume < 0.0f)
                        {
                            std::swap(tet.indices[1], tet.indices[2]);
                            volume = -volume;
                        }
                        tet.rest_volume = volume;
                        tet.compliance = 1.0e-7f;
                        tet.lambda = 0.0f;
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

        result.surface_to_particle.reserve(surface.vertices.size());
        const float grid_scale = static_cast<float>(grid_resolution - 1) * 0.5f;
        for (const vertex& surface_vertex : surface.vertices)
        {
            const float target_x = surface_vertex.position[0] * 0.94f;
            const float target_y = surface_vertex.position[1] * 0.94f;
            const float target_z = surface_vertex.position[2] * 0.94f;
            const auto to_grid = [grid_scale, grid_resolution](float value) -> std::uint32_t
            {
                const float scaled = (value + 1.0f) * grid_scale;
                return std::min(static_cast<std::uint32_t>(std::max(0.0f, std::round(scaled))), grid_resolution - 1);
            };
            const std::uint32_t center_x = to_grid(target_x);
            const std::uint32_t center_y = to_grid(target_y);
            const std::uint32_t center_z = to_grid(target_z);

            std::uint32_t best_index = 0;
            float best_distance = std::numeric_limits<float>::max();
            for (int radius = 0; radius <= 3 && best_distance == std::numeric_limits<float>::max(); ++radius)
            {
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    for (int dy = -radius; dy <= radius; ++dy)
                    {
                        for (int dx = -radius; dx <= radius; ++dx)
                        {
                            const int gx = static_cast<int>(center_x) + dx;
                            const int gy = static_cast<int>(center_y) + dy;
                            const int gz = static_cast<int>(center_z) + dz;
                            if (gx < 0 || gy < 0 || gz < 0
                                || gx >= static_cast<int>(grid_resolution)
                                || gy >= static_cast<int>(grid_resolution)
                                || gz >= static_cast<int>(grid_resolution)) continue;
                            const std::int32_t candidate = grid_to_particle[grid_index(
                                static_cast<std::uint32_t>(gx), static_cast<std::uint32_t>(gy), static_cast<std::uint32_t>(gz))];
                            if (candidate < 0) continue;
                            const simulation_particle& p = result.particles[static_cast<std::size_t>(candidate)];
                            const float dxp = p.position[0] - target_x;
                            const float dyp = p.position[1] - target_y;
                            const float dzp = p.position[2] - target_z;
                            const float distance = dxp * dxp + dyp * dyp + dzp * dzp;
                            if (distance < best_distance)
                            {
                                best_distance = distance;
                                best_index = static_cast<std::uint32_t>(candidate);
                            }
                        }
                    }
                }
            }
            result.surface_to_particle.push_back(best_index);
        }

        return result;
    }
}
