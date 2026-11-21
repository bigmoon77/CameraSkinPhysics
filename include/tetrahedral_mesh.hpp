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

    struct tetrahedral_mesh
    {
        std::vector<simulation_particle> particles;
        std::vector<float> inverse_masses;
        std::vector<tetrahedron> tetrahedra;
        std::vector<uint2> incidence_ranges;
        std::vector<std::uint32_t> incident_tetrahedra;
        std::vector<std::array<std::uint32_t, 4>> tetrahedron_indices;
        std::vector<std::uint32_t> surface_to_particle;
        std::vector<std::array<float, 3>> surface_offsets;
    };

    tetrahedral_mesh make_tetrahedral_mesh(const sphere_mesh& surface, std::uint32_t grid_resolution = 28);
}
