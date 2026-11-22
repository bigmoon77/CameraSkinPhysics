#include "shape_matching.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace csp
{
    shape_matching_clusters build_shape_matching_clusters(
        const sphere_mesh& mesh,
        const std::uint32_t target_core_vertices,
        const std::uint32_t overlap_rings)
    {
        if (mesh.vertices.empty())
            throw std::invalid_argument("Shape matching requires at least one vertex.");
        if (mesh.edges.empty() && mesh.vertices.size() > 1u)
            throw std::invalid_argument("Shape matching requires surface edge topology.");
        if (target_core_vertices == 0u)
            throw std::invalid_argument("Shape matching core size must be greater than zero.");
        if (mesh.vertices.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
            throw std::invalid_argument("Shape matching vertex count exceeds 32-bit indices.");

        const std::uint32_t vertex_count = static_cast<std::uint32_t>(mesh.vertices.size());
        std::vector<std::uint32_t> degree(vertex_count, 0u);
        for (const surface_edge& edge : mesh.edges)
        {
            if (edge.indices[0] >= vertex_count || edge.indices[1] >= vertex_count
                || edge.indices[0] == edge.indices[1])
                throw std::invalid_argument("Shape matching edge contains invalid indices.");
            ++degree[edge.indices[0]];
            ++degree[edge.indices[1]];
        }

        std::vector<mesh_uint2> adjacency_ranges(vertex_count);
        std::uint64_t adjacency_total = 0;
        for (std::uint32_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (adjacency_total > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("Shape matching adjacency exceeds 32-bit offsets.");
            adjacency_ranges[vertex_index] = {
                static_cast<std::uint32_t>(adjacency_total), degree[vertex_index]};
            adjacency_total += degree[vertex_index];
        }
        if (adjacency_total > std::numeric_limits<std::uint32_t>::max())
            throw std::invalid_argument("Shape matching adjacency exceeds 32-bit offsets.");

        std::vector<std::uint32_t> adjacency(static_cast<std::size_t>(adjacency_total));
        std::vector<std::uint32_t> adjacency_write(vertex_count, 0u);
        for (const surface_edge& edge : mesh.edges)
        {
            const std::uint32_t a = edge.indices[0];
            const std::uint32_t b = edge.indices[1];
            adjacency[adjacency_ranges[a].x + adjacency_write[a]++] = b;
            adjacency[adjacency_ranges[b].x + adjacency_write[b]++] = a;
        }

        std::vector<std::uint8_t> assigned(vertex_count, 0u);
        std::vector<std::vector<std::uint32_t>> core_clusters;
        std::vector<std::uint32_t> queue;
        queue.reserve(target_core_vertices);
        for (std::uint32_t seed = 0; seed < vertex_count; ++seed)
        {
            if (assigned[seed] != 0u)
                continue;

            queue.clear();
            queue.push_back(seed);
            assigned[seed] = 1u;
            std::size_t queue_read = 0;
            while (queue_read < queue.size() && queue.size() < target_core_vertices)
            {
                const std::uint32_t current = queue[queue_read++];
                const mesh_uint2 range = adjacency_ranges[current];
                for (std::uint32_t offset = 0; offset < range.y && queue.size() < target_core_vertices; ++offset)
                {
                    const std::uint32_t neighbor = adjacency[range.x + offset];
                    if (assigned[neighbor] != 0u)
                        continue;
                    assigned[neighbor] = 1u;
                    queue.push_back(neighbor);
                }
            }
            core_clusters.push_back(queue);
        }

        if (core_clusters.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::invalid_argument("Shape matching cluster count exceeds 32-bit indices.");

        shape_matching_clusters result{};
        result.cluster_vertex_ranges.resize(core_clusters.size());
        std::vector<std::uint32_t> membership_counts(vertex_count, 0u);
        std::vector<std::uint32_t> member_marks(vertex_count, 0u);
        std::vector<std::uint32_t> members;
        std::vector<std::uint32_t> frontier;

        for (std::uint32_t cluster_index = 0;
            cluster_index < static_cast<std::uint32_t>(core_clusters.size()); ++cluster_index)
        {
            members = core_clusters[cluster_index];
            std::fill(member_marks.begin(), member_marks.end(), 0u);
            const std::uint32_t mark = cluster_index + 1u;
            for (const std::uint32_t vertex_index : members)
                member_marks[vertex_index] = mark;
            frontier = members;

            for (std::uint32_t ring = 0; ring < overlap_rings; ++ring)
            {
                std::vector<std::uint32_t> next_frontier;
                for (const std::uint32_t current : frontier)
                {
                    const mesh_uint2 range = adjacency_ranges[current];
                    for (std::uint32_t offset = 0; offset < range.y; ++offset)
                    {
                        const std::uint32_t neighbor = adjacency[range.x + offset];
                        if (member_marks[neighbor] == mark)
                            continue;
                        member_marks[neighbor] = mark;
                        members.push_back(neighbor);
                        next_frontier.push_back(neighbor);
                    }
                }
                frontier.swap(next_frontier);
                if (frontier.empty())
                    break;
            }

            if (result.cluster_vertices.size() > std::numeric_limits<std::uint32_t>::max()
                || members.size() > std::numeric_limits<std::uint32_t>::max() - result.cluster_vertices.size())
                throw std::invalid_argument("Shape matching cluster membership exceeds 32-bit offsets.");

            result.cluster_vertex_ranges[cluster_index] = {
                static_cast<std::uint32_t>(result.cluster_vertices.size()),
                static_cast<std::uint32_t>(members.size())};
            result.cluster_vertices.insert(result.cluster_vertices.end(), members.begin(), members.end());
            for (const std::uint32_t vertex_index : members)
                ++membership_counts[vertex_index];
        }

        result.vertex_cluster_ranges.resize(vertex_count);
        std::uint64_t membership_total = 0;
        for (std::uint32_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (membership_total > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("Shape matching vertex memberships exceed 32-bit offsets.");
            result.vertex_cluster_ranges[vertex_index] = {
                static_cast<std::uint32_t>(membership_total), membership_counts[vertex_index]};
            membership_total += membership_counts[vertex_index];
        }
        if (membership_total > std::numeric_limits<std::uint32_t>::max())
            throw std::invalid_argument("Shape matching vertex memberships exceed 32-bit offsets.");

        result.vertex_clusters.resize(static_cast<std::size_t>(membership_total));
        std::vector<std::uint32_t> membership_write(vertex_count, 0u);
        for (std::uint32_t cluster_index = 0;
            cluster_index < static_cast<std::uint32_t>(result.cluster_vertex_ranges.size()); ++cluster_index)
        {
            const mesh_uint2 range = result.cluster_vertex_ranges[cluster_index];
            for (std::uint32_t offset = 0; offset < range.y; ++offset)
            {
                const std::uint32_t vertex_index = result.cluster_vertices[range.x + offset];
                const std::uint32_t destination =
                    result.vertex_cluster_ranges[vertex_index].x + membership_write[vertex_index]++;
                result.vertex_clusters[destination] = cluster_index;
            }
        }

        return result;
    }
}
