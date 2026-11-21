#include "sphere_mesh.hpp"
#include "tetrahedral_mesh.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

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
    check(surface.indices.size() == 597600, "default sphere index count must match its grid topology");
    check(surface.indices.size() % 3 == 0, "sphere index count must be divisible by three");

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
    check(volume.surface_to_particle.size() == surface.vertices.size(), "every render vertex must map to a simulation particle");
    check(volume.surface_offsets.size() == surface.vertices.size(), "every render vertex must have a rest offset");

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

    for (std::size_t index = 0; index < volume.surface_to_particle.size(); ++index)
    {
        check(volume.surface_to_particle[index] < volume.particles.size(), "surface mapping must reference a valid particle");
        if (volume.surface_to_particle[index] < volume.incidence_ranges.size())
        {
            check(volume.incidence_ranges[volume.surface_to_particle[index]].y > 0,
                "surface mapping must reference a particle connected to at least one tetrahedron");
        }
        const auto& offset = volume.surface_offsets[index];
        check(finite_value(offset[0]) && finite_value(offset[1]) && finite_value(offset[2]),
            "surface rest offsets must be finite");
    }

    std::cout << "surface_vertices=" << surface.vertices.size()
        << " surface_indices=" << surface.indices.size()
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
