#include "sphere_mesh.hpp"
#include "tetrahedral_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>\n#include <vector>

namespace
{
    int failures = 0;

    void check(bool condition, const std::string& message)
    {
        if (!condition)
        {
            std::cerr << "[FAIL] " << message << '\n';
            ++failures;
        }
    }

    bool finite_value(const float value)
    {
        return std::isfinite(value);
    }

    float dot3(const float* a, const float* b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
}

int main()
{
    const csp::sphere_mesh surface = csp::make_sphere_mesh();
    check(surface.vertices.size() == 100000, "default sphere must contain exactly 100,000 vertices");
    check(surface.indices.size() == 599988, "default sphere index count must match its grid topology");
    check(surface.indices.size() % 3 == 0, "sphere index count must be divisible by three");

    check(surface.edges.size() == 299994, "default sphere must contain 299,994 unique surface edges");
    check(surface.edge_incidence_ranges.size() == surface.vertices.size(),
        "surface edge incidence range count must match vertex count");
    check(surface.incident_corrections.size() == surface.edges.size() * 2,
        "each surface edge must have exactly two incidence entries");

    bool surface_edges_valid = true;
    for (std::size_t edge_index = 0; edge_index < surface.edges.size(); ++edge_index)
    {
        const csp::surface_edge& edge = surface.edges[edge_index];
        if (edge.indices[0] >= surface.vertices.size() || edge.indices[1] >= surface.vertices.size()
            || edge.indices[0] == edge.indices[1] || !(edge.rest_length > 0.0f)
            || !finite_value(edge.rest_length) || !(edge.compliance >= 0.0f)
            || !finite_value(edge.compliance))
        {
            surface_edges_valid = false;
            break;
        }
    }
    check(surface_edges_valid, "surface edge indices and rest lengths must be valid");

    bool surface_edges_sorted_unique = true;
    for (std::size_t edge_index = 1; edge_index < surface.edges.size(); ++edge_index)
    {
        const csp::surface_edge& previous = surface.edges[edge_index - 1];
        const csp::surface_edge& current = surface.edges[edge_index];
        if (previous.indices[0] > current.indices[0]
            || (previous.indices[0] == current.indices[0]
                && previous.indices[1] >= current.indices[1]))
        {
            surface_edges_sorted_unique = false;
            break;
        }
    }
    check(surface_edges_sorted_unique, "surface edges must be sorted and unique");

    const csp::sphere_mesh coarse_surface = csp::make_sphere_mesh(8, 12);
    check(coarse_surface.vertices.size() == 96, "coarse sphere vertex count must match its resolution");
    check(coarse_surface.indices.size() == 564, "coarse sphere index count must match its resolution");
    check(coarse_surface.edges.size() == 282, "coarse sphere edge count must match its resolution");
    check(coarse_surface.incident_corrections.size() == 564,
        "coarse sphere CSR must contain two entries per unique edge");

    std::uint64_t coarse_incidence_total = 0;
    bool coarse_incidence_valid = true;
    for (std::size_t vertex_index = 0; vertex_index < coarse_surface.edge_incidence_ranges.size(); ++vertex_index)
    {
        const csp::mesh_uint2 range = coarse_surface.edge_incidence_ranges[vertex_index];
        if (static_cast<std::size_t>(range.x) + range.y > coarse_surface.incident_corrections.size())
        {
            coarse_incidence_valid = false;
            break;
        }
        coarse_incidence_total += range.y;
        for (std::uint32_t offset = 0; offset < range.y; ++offset)
        {
            const std::uint32_t correction_index = coarse_surface.incident_corrections[range.x + offset];
            if (correction_index >= coarse_surface.edges.size() * 2u)
            {
                coarse_incidence_valid = false;
                break;
            }
            const csp::surface_edge& edge = coarse_surface.edges[correction_index / 2u];
            const std::uint32_t endpoint = correction_index % 2u;
            if (edge.indices[endpoint] != vertex_index)
            {
                coarse_incidence_valid = false;
                break;
            }
        }
        if (!coarse_incidence_valid) break;
    }
    check(coarse_incidence_valid, "coarse sphere edge CSR must reference incident edges");
    check(coarse_incidence_total == coarse_surface.edges.size() * 2ull,
        "coarse sphere CSR incidence count must match its edge count");

    std::uint64_t surface_incidence_total = 0;
    bool surface_incidence_valid = true;
    for (std::size_t vertex_index = 0; vertex_index < surface.edge_incidence_ranges.size(); ++vertex_index)
    {
        const csp::mesh_uint2 range = surface.edge_incidence_ranges[vertex_index];
        if (static_cast<std::size_t>(range.x) + range.y > surface.incident_corrections.size())
        {
            surface_incidence_valid = false;
            break;
        }
        surface_incidence_total += range.y;
        for (std::uint32_t offset = 0; offset < range.y; ++offset)
        {
            const std::uint32_t correction_index = surface.incident_corrections[range.x + offset];
            if (correction_index >= surface.edges.size() * 2u)
            {
                surface_incidence_valid = false;
                break;
            }
            const csp::surface_edge& edge = surface.edges[correction_index / 2u];
            const std::uint32_t endpoint = correction_index % 2u;
            if (edge.indices[endpoint] != vertex_index)
            {
                surface_incidence_valid = false;
                break;
            }
        }
        if (!surface_incidence_valid) break;
    }
    check(surface_incidence_valid, "surface edge CSR entries must reference incident edges");
    check(surface_incidence_total == surface.edges.size() * 2ull,
        "surface edge CSR must contain exactly two entries per edge");

    check(surface.gather_vertex_indices.size() == surface.vertices.size(),
        "gather vertex order must contain every surface vertex");
    std::vector<bool> gather_vertex_seen(surface.vertices.size(), false);
    bool gather_order_valid = true;
    std::uint32_t previous_degree = 0;
    std::uint32_t minimum_degree = UINT32_MAX;
    std::uint32_t maximum_degree = 0;
    std::uint32_t low_degree_count = 0;
    for (std::size_t order_index = 0; order_index < surface.gather_vertex_indices.size(); ++order_index)
    {
        const std::uint32_t vertex_index = surface.gather_vertex_indices[order_index];
        if (vertex_index >= surface.vertices.size() || gather_vertex_seen[vertex_index])
        {
            gather_order_valid = false;
            break;
        }
        gather_vertex_seen[vertex_index] = true;
        const std::uint32_t degree = surface.edge_incidence_ranges[vertex_index].y;
        if (order_index > 0 && degree < previous_degree)
        {
            gather_order_valid = false;
            break;
        }
        previous_degree = degree;
        minimum_degree = std::min(minimum_degree, degree);
        maximum_degree = std::max(maximum_degree, degree);
        if (degree <= 8u) ++low_degree_count;
    }
    check(gather_order_valid, "gather vertices must be a unique ascending-degree permutation");
    check(low_degree_count > 0, "gather order must contain low-degree vertices");
    check(maximum_degree > 8u, "default sphere must exercise the high-degree gather path");

    for (std::size_t index = 0; index < surface.vertices.size(); ++index)
    {
        const csp::vertex& value = surface.vertices[index];
        check(finite_value(value.position[0]) && finite_value(value.position[1]) && finite_value(value.position[2]),
            "sphere positions must be finite");
        check(finite_value(value.normal[0]) && finite_value(value.normal[1]) && finite_value(value.normal[2]),
            "sphere normals must be finite");
        const float normal_length = std::sqrt(dot3(value.normal, value.normal));
        check(std::abs(normal_length - 1.0f) < 1.0e-4f, "sphere normals must be normalized");
        if (index > 0 && failures > 20) break;
    }

    bool indices_valid = true;
    bool winding_valid = true;
    for (std::size_t index = 0; index < surface.indices.size(); index += 3)
    {
        const std::uint32_t ia = surface.indices[index];
        const std::uint32_t ib = surface.indices[index + 1];
        const std::uint32_t ic = surface.indices[index + 2];
        if (ia >= surface.vertices.size() || ib >= surface.vertices.size() || ic >= surface.vertices.size())
        {
            indices_valid = false;
            break;
        }
        const csp::vertex& a = surface.vertices[ia];
        const csp::vertex& b = surface.vertices[ib];
        const csp::vertex& c = surface.vertices[ic];
        const float ab[3] = {b.position[0] - a.position[0], b.position[1] - a.position[1], b.position[2] - a.position[2]};
        const float ac[3] = {c.position[0] - a.position[0], c.position[1] - a.position[1], c.position[2] - a.position[2]};
        const float cross[3] = {
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]
        };
        const float center[3] = {
            a.position[0] + b.position[0] + c.position[0],
            a.position[1] + b.position[1] + c.position[1],
            a.position[2] + b.position[2] + c.position[2]
        };
        if (dot3(cross, center) <= 0.0f) winding_valid = false;
    }
    check(indices_valid, "all sphere indices must be in range");
    check(winding_valid, "sphere triangles must have outward-facing winding");

    const csp::tetrahedral_mesh volume = csp::make_tetrahedral_mesh(surface);
    check(!volume.particles.empty(), "tetrahedral particle set must not be empty");
    check(!volume.tetrahedra.empty(), "tetrahedral element set must not be empty");
    check(volume.inverse_masses.size() == volume.particles.size(), "inverse mass count must match particle count");
    check(volume.incidence_ranges.size() == volume.particles.size(), "incidence range count must match particle count");
    check(volume.surface_mappings.size() == surface.vertices.size(), "every render vertex must have a tetrahedral skin mapping");

    for (std::size_t index = 0; index < volume.particles.size(); ++index)
    {
        const csp::simulation_particle& particle = volume.particles[index];
        check(finite_value(particle.position[0]) && finite_value(particle.position[1]) && finite_value(particle.position[2]),
            "simulation particle positions must be finite");
        check(particle.position[0] * particle.position[0] + particle.position[1] * particle.position[1]
            + particle.position[2] * particle.position[2] <= 1.00001f, "simulation particles must lie inside the sphere");
        check(volume.inverse_masses[index] > 0.0f && finite_value(volume.inverse_masses[index]),
            "dynamic particle inverse masses must be finite and positive");
    }

    std::uint64_t incidence_total = 0;
    for (std::size_t particle_index = 0; particle_index < volume.incidence_ranges.size(); ++particle_index)
    {
        const csp::uint2 range = volume.incidence_ranges[particle_index];
        check(static_cast<std::size_t>(range.x) + range.y <= volume.incident_tetrahedra.size(),
            "particle incidence range must stay within CSR storage");
        incidence_total += range.y;
        for (std::uint32_t offset = 0; offset < range.y; ++offset)
        {
            const std::uint32_t tet_index = volume.incident_tetrahedra[range.x + offset];
            check(tet_index < volume.tetrahedra.size(), "incident tetrahedron index must be valid");
            if (tet_index >= volume.tetrahedra.size()) continue;
            const csp::tetrahedron& tet = volume.tetrahedra[tet_index];
            bool contains_particle = false;
            for (const std::uint32_t id : tet.indices) contains_particle = contains_particle || id == particle_index;
            check(contains_particle, "CSR incidence must reference a tetrahedron containing its particle");
        }
    }
    check(incidence_total == volume.tetrahedra.size() * 4ull, "each tetrahedron must contribute exactly four incidences");

    for (std::size_t index = 0; index < volume.tetrahedra.size(); ++index)
    {
        const csp::tetrahedron& tet = volume.tetrahedra[index];
        check(tet.rest_volume > 0.0f && finite_value(tet.rest_volume), "tetrahedron rest volume must be finite and positive");
        for (const std::uint32_t id : tet.indices)
            check(id < volume.particles.size(), "tetrahedron particle index must be valid");
    }

    for (std::size_t index = 0; index < volume.surface_mappings.size(); ++index)
    {
        const csp::surface_mapping& mapping = volume.surface_mappings[index];
        float weight_sum = 0.0f;
        float reconstructed_position[3] = {
            mapping.rest_offset[0], mapping.rest_offset[1], mapping.rest_offset[2]
        };
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            check(mapping.indices[corner] < volume.particles.size(), "skin mapping particle index must be valid");
            check(finite_value(mapping.weights[corner]), "skin mapping weights must be finite");
            check(mapping.weights[corner] >= -1.0e-4f && mapping.weights[corner] <= 1.0001f,
                "skin mapping weights must be inside the tetrahedron");
            weight_sum += mapping.weights[corner];
            if (mapping.indices[corner] < volume.particles.size())
            {
                const csp::simulation_particle& particle = volume.particles[mapping.indices[corner]];
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                    reconstructed_position[axis] += mapping.weights[corner] * particle.position[axis];
            }
        }
        check(std::abs(weight_sum - 1.0f) < 1.0e-3f, "barycentric skin weights must sum to one");
        check(finite_value(mapping.rest_offset[0]) && finite_value(mapping.rest_offset[1])
            && finite_value(mapping.rest_offset[2]), "skin rest offsets must be finite");
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            check(std::abs(reconstructed_position[axis] - surface.vertices[index].position[axis]) < 1.0e-4f,
                "skin mapping must reproduce the undeformed surface exactly");
        }
    }

    std::cout << "surface_vertices=" << surface.vertices.size()
        << " surface_indices=" << surface.indices.size()
        << " surface_min_degree=" << minimum_degree
        << " surface_max_degree=" << maximum_degree
        << " surface_degree_le_8=" << low_degree_count
        << " simulation_particles=" << volume.particles.size()
        << " tetrahedra=" << volume.tetrahedra.size()
        << " incidence_entries=" << volume.incident_tetrahedra.size() << '\n';

    if (failures != 0)
    {
        std::cerr << failures << " validation checks failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "All geometry validation checks passed.\n";
    return EXIT_SUCCESS;
}
