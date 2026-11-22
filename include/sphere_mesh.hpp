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

    struct triangle_area_constraint
    {
        std::uint32_t indices[3];
        float rest_area;
        float compliance;
    };

    struct bending_constraint
    {
        // Shared edge endpoints followed by the opposite vertex of each face.
        std::uint32_t indices[4];
        float rest_distance;
        float compliance;
    };

    struct surface_topology_info
    {
        std::uint32_t boundary_edge_count = 0;
        std::uint32_t non_manifold_edge_count = 0;
        std::uint32_t orientation_conflict_count = 0;
        float signed_rest_volume = 0.0f;
        bool closed_oriented = false;
    };

    struct sphere_mesh
    {
        std::vector<vertex> vertices;
        std::vector<std::uint32_t> indices;
        std::vector<surface_edge> edges;
        std::vector<mesh_uint2> edge_incidence_ranges;
        std::vector<std::uint32_t> incident_corrections;
        std::vector<triangle_area_constraint> triangle_areas;
        std::vector<mesh_uint2> triangle_incidence_ranges;
        // Encodes triangle_index * 3 + corner_index for each incident vertex.
        std::vector<std::uint32_t> incident_triangle_corners;
        std::vector<bending_constraint> bending;
        std::vector<mesh_uint2> bending_incidence_ranges;
        // Encodes bending_constraint_index * 4 + corner_index.
        std::vector<std::uint32_t> incident_bending_corners;
        surface_topology_info topology;
    };

    static_assert(sizeof(vertex) == 24);
    static_assert(sizeof(mesh_uint2) == 8);
    static_assert(sizeof(surface_edge) == 16);
    static_assert(sizeof(triangle_area_constraint) == 20);
    static_assert(sizeof(bending_constraint) == 24);

    // Builds all surface constraints and adjacency from arbitrary indexed triangle geometry.
    // Vertices and indices must already be populated; triangle winding must be consistent
    // for signed volume to be meaningful.
    void build_surface_constraints(sphere_mesh& mesh,
        float edge_compliance = 2.0e-6f,
        float area_compliance = 1.0e-7f,
        float bending_compliance = 2.0e-4f);

    sphere_mesh make_sphere_mesh(std::uint32_t ring_count = 250, std::uint32_t segment_count = 400);
}
