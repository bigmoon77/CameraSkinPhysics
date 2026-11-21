#pragma once
#include <cstdint>
#include <vector>
namespace csp
{
    struct vertex { float position[3]; float normal[3]; };
    struct sphere_mesh { std::vector<vertex> vertices; std::vector<std::uint32_t> indices; };
    static_assert(sizeof(vertex) == 24);

    sphere_mesh make_sphere_mesh(std::uint32_t ring_count = 250, std::uint32_t segment_count = 400);
}
