#pragma once

#include "sphere_mesh.hpp"

#include <cstdint>
#include <vector>

namespace csp
{
    struct shape_matching_clusters
    {
        std::vector<mesh_uint2> cluster_vertex_ranges;
        std::vector<std::uint32_t> cluster_vertices;
        std::vector<mesh_uint2> vertex_cluster_ranges;
        std::vector<std::uint32_t> vertex_clusters;
    };

    // Partitions a triangle surface into connected geodesic patches and expands each
    // patch by overlap_rings to avoid hard seams between independently fitted clusters.
    shape_matching_clusters build_shape_matching_clusters(
        const sphere_mesh& mesh,
        std::uint32_t target_core_vertices = 128,
        std::uint32_t overlap_rings = 1);
}
