#pragma once
#include <cstdint>
#include <vector>

namespace csp
{
    struct vertex
    {
        float position[3];
        float normal[3];
    };

    struct mesh_uint2
    {
        std::uint32_t x;
        std::uint32_t y;
    };

    struct surface_edge
    {
        std::uint32_t indices[2];
        float rest_length;
        float compliance;
    };

    struct sphere_mesh
    {
        std::vector<vertex> vertices;
        std::vector<std::uint32_t> indices;
        std::vector<surface_edge> edges;
        std::vector<mesh_uint2> edge_incidence_ranges;
        std::vector<std::uint32_t> incident_edges;
    };

    static_assert(sizeof(vertex) == 24);
    static_assert(sizeof(mesh_uint2) == 8);
    static_assert(sizeof(surface_edge) == 16);

    sphere_mesh make_sphere_mesh(std::uint32_t ring_count = 250, std::uint32_t segment_count = 400);
}
