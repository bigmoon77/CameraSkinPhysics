#pragma once
#include "sphere_mesh.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace csp
{
    struct simulation_particle
    {
        float position[4];
        float velocity[4];
    };

    struct tetrahedron
    {
        std::uint32_t indices[4];
        float rest_volume;
        float compliance;
        float lambda;
        float padding;
    };

    struct uint2
    {
        std::uint32_t x;
        std::uint32_t y;
    };

    struct surface_mapping
    {
        std::uint32_t indices[4];
        float weights[4];
        float rest_offset[4];
    };

    struct tetrahedral_mesh
    {
        std::vector<simulation_particle> particles;
        std::vector<float> inverse_masses;
        std::vector<tetrahedron> tetrahedra;
        std::vector<uint2> incidence_ranges;
        std::vector<std::uint32_t> incident_tetrahedra;
        std::vector<std::array<std::uint32_t, 4>> tetrahedron_indices;
        std::vector<surface_mapping> surface_mappings;
    };

    static_assert(sizeof(simulation_particle) == 32);
    static_assert(sizeof(tetrahedron) == 32);
    static_assert(sizeof(surface_mapping) == 48);

    tetrahedral_mesh make_tetrahedral_mesh(const sphere_mesh& surface, std::uint32_t grid_resolution = 28);
}
