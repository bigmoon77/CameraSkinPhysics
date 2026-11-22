#pragma once
#include "sphere_mesh.hpp"
#include <filesystem>

namespace csp
{
    sphere_mesh load_obj_mesh(const std::filesystem::path& path);
}
