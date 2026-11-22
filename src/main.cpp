#include "d3d12_renderer.hpp"
#include "sphere_mesh.hpp"
#include "tetrahedral_mesh.hpp"
#include "obj_mesh.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <windows.h>
#include <commctrl.h>
#include <array>
#include <sstream>
#include <iomanip>
#include <cwchar>
#include <exception>
#include <string>
#include <filesystem>
#include <fstream>
#include <commdlg.h>
#include <shellapi.h>

namespace
{
    constexpr wchar_t window_class_name[] = L"CameraSkinPhysicsWindow";
    constexpr wchar_t parameter_window_class_name[] = L"CameraSkinPhysicsParameters";
    constexpr int parameter_control_base = 100;
    constexpr int parameter_reset_id = 200;
    constexpr int parameter_pause_id = 201;
    constexpr int parameter_model_id = 202;
    constexpr int parameter_restart_id = 203;
    constexpr int parameter_count = 12;
    HWND parameter_sliders[parameter_count]{};
    HWND parameter_value_labels[parameter_count]{};
    HWND parameter_window = nullptr;
    HWND performance_label = nullptr;
    HWND pause_button = nullptr;
    csp::d3d12_renderer* active_renderer = nullptr;
    bool benchmark_running = false;
    std::filesystem::path active_obj_path;
    std::filesystem::path active_skin_path;
    bool custom_model_loaded = false;
    int parameter_scroll_position = 0;
    int parameter_content_height = 0;

    BOOL CALLBACK scroll_parameter_children(HWND child, LPARAM delta_value)
    {
        RECT rect{};
        if (GetWindowRect(child, &rect))
        {
            POINT position{rect.left, rect.top};
            MapWindowPoints(HWND_DESKTOP, GetParent(child), &position, 1);
            const int delta = static_cast<int>(delta_value);
            SetWindowPos(child, nullptr, position.x, position.y - delta, 0, 0,
                SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOCOPYBITS);
        }
        return TRUE;
    }

    void update_parameter_scrollbar(HWND window)
    {
        RECT client{};
        GetClientRect(window, &client);
        const int page_height = std::max(0L, client.bottom - client.top);
        const int maximum_position = std::max(0, parameter_content_height - page_height);
        parameter_scroll_position = std::clamp(parameter_scroll_position, 0, maximum_position);
        SCROLLINFO info{};
        info.cbSize = sizeof(info);
        info.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;
        info.nMin = 0;
        info.nMax = std::max(0, parameter_content_height - 1);
        info.nPage = static_cast<UINT>(page_height);
        info.nPos = parameter_scroll_position;
        SetScrollInfo(window, SB_VERT, &info, TRUE);
    }

    void scroll_parameter_window(HWND window, int requested_position)
    {
        RECT client{};
        GetClientRect(window, &client);
        const int page_height = std::max(0L, client.bottom - client.top);
        const int maximum_position = std::max(0, parameter_content_height - page_height);
        const int next_position = std::clamp(requested_position, 0, maximum_position);
        const int delta = next_position - parameter_scroll_position;
        if (delta == 0) return;
        parameter_scroll_position = next_position;
        EnumChildWindows(window, scroll_parameter_children, static_cast<LPARAM>(delta));
        update_parameter_scrollbar(window);
    }

    struct parameter_description
    {
        const wchar_t* label;
        float minimum;
        float maximum;
        bool integer;
        bool logarithmic = false;
    };

    constexpr parameter_description parameter_descriptions[parameter_count] = {
        {L"\u53cd\u5fa9\u56de\u6570", 1.0f, static_cast<float>(csp::max_solver_iterations), true},
        {L"\u30a8\u30c3\u30b8\u5236\u7d04\u306e\u7de9\u548c\u7387", 0.0f, 1.5f, false},
        {L"\u88dc\u6b63\u91cf\u306e\u4e0a\u9650", 0.0001f, 0.05f, false},
        {L"\u7c92\u5b50\u306e\u534a\u5f84", 0.001f, 0.05f, false},
        {L"\u63a5\u89e6\u6642\u306e\u62bc\u3057\u51fa\u3057\u91cf", 0.0f, 0.03f, false},
        {L"\u63a5\u89e6\u6642\u306e\u6469\u64e6", 0.0f, 1.0f, false},
        {L"\u53cd\u767a\u4fc2\u6570", 0.0f, 1.0f, false},
        {L"\u63a5\u89e6\u5224\u5b9a\u306e\u5e45", 0.005f, 0.5f, false},
        {L"\u30a8\u30c3\u30b8\u9577\u306e\u30b3\u30f3\u30d7\u30e9\u30a4\u30a2\u30f3\u30b9", 1.0e-9f, 1.0e-2f, false, true},
        {L"\u9762\u7a4d\u306e\u30b3\u30f3\u30d7\u30e9\u30a4\u30a2\u30f3\u30b9", 1.0e-9f, 1.0e-2f, false, true},
        {L"\u66f2\u3052\u306e\u30b3\u30f3\u30d7\u30e9\u30a4\u30a2\u30f3\u30b9", 1.0e-9f, 1.0e-2f, false, true},
        {L"\u30af\u30e9\u30b9\u30bf\u5f62\u72b6\u306e\u5fa9\u5143\u5f37\u5ea6", 0.0f, 1.0f, false}
    };

    float get_parameter_value(const csp::simulation_parameters& parameters, int index)
    {
        switch (index)
        {
        case 0: return static_cast<float>(parameters.solver_iterations);
        case 1: return parameters.edge_relaxation;
        case 2: return parameters.max_correction;
        case 3: return parameters.particle_radius;
        case 4: return parameters.pushout;
        case 5: return parameters.velocity_friction;
        case 6: return parameters.restitution;
        case 8: return parameters.edge_compliance;
        case 9: return parameters.area_compliance;
        case 10: return parameters.bending_compliance;
        case 11: return parameters.shape_matching_strength;
        default: return parameters.contact_band;
        }
    }

    void set_parameter_value(csp::simulation_parameters& parameters, int index, float value)
    {
        switch (index)
        {
        case 0: parameters.solver_iterations = static_cast<std::uint32_t>(value); break;
        case 1: parameters.edge_relaxation = value; break;
        case 2: parameters.max_correction = value; break;
        case 3: parameters.particle_radius = value; break;
        case 4: parameters.pushout = value; break;
        case 5: parameters.velocity_friction = value; break;
        case 6: parameters.restitution = value; break;
        case 8: parameters.edge_compliance = value; break;
        case 9: parameters.area_compliance = value; break;
        case 10: parameters.bending_compliance = value; break;
        case 11: parameters.shape_matching_strength = value; break;
        default: parameters.contact_band = value; break;
        }
    }

    void update_parameter_controls()
    {
        if (active_renderer == nullptr) return;
        const csp::simulation_parameters parameters = active_renderer->get_simulation_parameters();
        for (int index = 0; index < parameter_count; ++index)
        {
            const float value = get_parameter_value(parameters, index);
            const parameter_description& description = parameter_descriptions[index];
            const float normalized = description.logarithmic
                ? (std::log(value) - std::log(description.minimum))
                    / (std::log(description.maximum) - std::log(description.minimum))
                : (value - description.minimum) / (description.maximum - description.minimum);
            const int position = static_cast<int>(std::lround(normalized * 1000.0f));
            SendMessageW(parameter_sliders[index], TBM_SETPOS, TRUE, position);
            std::wostringstream text;
            if (description.logarithmic)
                text << std::scientific << std::setprecision(2) << value;
            else
                text << std::fixed << std::setprecision(index == 0 ? 0 : 4) << value;
            SetWindowTextW(parameter_value_labels[index], text.str().c_str());
        }
    }

    void update_performance_controls()
    {
        if (active_renderer == nullptr || performance_label == nullptr) return;
        const csp::performance_metrics metrics = active_renderer->get_performance_metrics();
        std::wostringstream text;
        text << std::fixed << std::setprecision(2)
            << L"\u525b\u6027\u8abf\u6574: Compliance\u306f\u5024\u304c\u5c0f\u3055\u3044\u307b\u3069\u786c\u3044\n"
            << L"CPU\u30d5\u30ec\u30fc\u30e0 " << metrics.cpu_frame_ms << L" ms    GPU\u5408\u8a08 " << metrics.gpu_total_ms << L" ms\n"
            << L"\u30b7\u30df\u30e5\u30ec\u30fc\u30b7\u30e7\u30f3 " << metrics.gpu_simulation_ms << L" ms    \u63a5\u89e6\u51e6\u7406 "
            << metrics.gpu_collision_ms << L" ms    \u63cf\u753b " << metrics.gpu_render_ms << L" ms\n"
            << L"\u5236\u7d04\u8a08\u7b97 " << metrics.camera_constraint_solve_ms << L" ms    \u88dc\u6b63\u96c6\u7d04 "
            << metrics.camera_gather_ms << L" ms";
        SetWindowTextW(performance_label, text.str().c_str());
    }

    void apply_parameter_control(HWND slider)
    {
        if (active_renderer == nullptr || slider == nullptr) return;
        const int index = GetDlgCtrlID(slider) - parameter_control_base;
        if (index < 0 || index >= parameter_count) return;
        const parameter_description& description = parameter_descriptions[index];
        const int position = static_cast<int>(SendMessageW(slider, TBM_GETPOS, 0, 0));
        const float normalized = static_cast<float>(position) / 1000.0f;
        const float value = description.logarithmic
            ? std::exp(std::log(description.minimum)
                + normalized * (std::log(description.maximum) - std::log(description.minimum)))
            : (description.integer
                ? std::round(description.minimum + normalized * (description.maximum - description.minimum))
                : description.minimum + normalized * (description.maximum - description.minimum));
        csp::simulation_parameters parameters = active_renderer->get_simulation_parameters();
        set_parameter_value(parameters, index, value);
        active_renderer->set_simulation_parameters(parameters);
        update_parameter_controls();
    }

    
    csp::tetrahedral_mesh make_surface_only_volume(const csp::sphere_mesh& mesh)
    {
        csp::tetrahedral_mesh volume{};
        const float positions[4][3] = {
            {-0.01f, -0.01f, -0.01f}, {0.01f, -0.01f, -0.01f},
            {0.0f, 0.01f, -0.01f}, {0.0f, 0.0f, 0.01f}
        };
        for (const auto& position : positions)
        {
            csp::simulation_particle particle{};
            particle.position[0] = position[0];
            particle.position[1] = position[1];
            particle.position[2] = position[2];
            particle.position[3] = 1.0f;
            volume.particles.push_back(particle);
            volume.inverse_masses.push_back(1.0f);
        }
        csp::tetrahedron tetra{};
        tetra.indices[0] = 0; tetra.indices[1] = 1;
        tetra.indices[2] = 2; tetra.indices[3] = 3;
        tetra.rest_volume = 1.0e-6f;
        volume.tetrahedra.push_back(tetra);
        volume.incidence_ranges.resize(4);
        for (std::uint32_t index = 0; index < 4; ++index)
        {
            volume.incidence_ranges[index] = {index, 1};
            volume.incident_tetrahedra.push_back(0);
        }
        volume.tetrahedron_indices.push_back({0, 1, 2, 3});
        volume.surface_mappings.resize(mesh.vertices.size());
        for (csp::surface_mapping& mapping : volume.surface_mappings)
        {
            mapping.indices[0] = 0;
            mapping.weights[0] = 1.0f;
        }
        return volume;
    }

    bool export_blend_to_obj(const std::filesystem::path& blend_path,
        std::filesystem::path& obj_path, std::filesystem::path& skin_path)
    {
        wchar_t blender_path[MAX_PATH]{};
        if (SearchPathW(nullptr, L"blender.exe", nullptr, MAX_PATH, blender_path, nullptr) == 0)
        {
            MessageBoxW(parameter_window,
                L"blender.exe \u304c PATH \u306b\u898b\u3064\u304b\u308a\u307e\u305b\u3093\u3002blender.exe \u304c\u3042\u308b\u30d5\u30a9\u30eb\u30c0\u30fc\u3092 PATH \u306b\u8ffd\u52a0\u3057\u3066\u304f\u3060\u3055\u3044\u3002",
                L"\u30e2\u30c7\u30eb\u8aad\u307f\u8fbc\u307f", MB_ICONERROR | MB_OK);
            return false;
        }

        wchar_t temporary_directory[MAX_PATH]{};
        if (GetTempPathW(MAX_PATH, temporary_directory) == 0) return false;
        const std::filesystem::path temp = temporary_directory;
        const std::filesystem::path script_path = temp / L"csp_blend_export.py";
        obj_path = temp / L"csp_blend_import.obj";
        skin_path = temp / L"csp_blend_import.cspanim";
        std::ofstream script(script_path, std::ios::binary);
        if (!script) return false;
        script << R"PY(import bpy
import sys
import json
from mathutils import Matrix

args = sys.argv[sys.argv.index("--") + 1:]
obj_path, skin_path = args
scene = bpy.context.scene
mesh_objects = [obj for obj in scene.objects if obj.type == "MESH"]
armatures = []
for obj in mesh_objects:
    for modifier in obj.modifiers:
        if modifier.type == "ARMATURE" and modifier.object is not None:
            if modifier.object not in armatures:
                armatures.append(modifier.object)
if len(armatures) > 1:
    raise RuntimeError("Only one shared Armature is supported per imported model.")
armature = armatures[0] if armatures else None
bones = list(armature.data.bones) if armature is not None else []
if len(bones) > 512:
    raise RuntimeError("The imported Armature exceeds 512 bones.")
bone_indices = {bone.name: index for index, bone in enumerate(bones)}

vertex_weights = []
vertex_offset = 0
with open(obj_path, "w", encoding="utf-8") as obj_file:
    for obj in mesh_objects:
        mesh = obj.data
        mesh.calc_loop_triangles()
        world = obj.matrix_world
        for vertex in mesh.vertices:
            point = world @ vertex.co
            obj_file.write("v %.9g %.9g %.9g\n" % (point.x, point.z, -point.y))
            influences = []
            if armature is not None:
                for group in vertex.groups:
                    group_name = obj.vertex_groups[group.group].name
                    bone_index = bone_indices.get(group_name)
                    if bone_index is not None and group.weight > 0.0:
                        influences.append((bone_index, group.weight))
            influences.sort(key=lambda item: item[1], reverse=True)
            influences = influences[:4]
            total = sum(weight for _, weight in influences)
            if total > 1.0e-8:
                influences = [(index, weight / total) for index, weight in influences]
            else:
                influences = []
            while len(influences) < 4:
                influences.append((0, 0.0))
            vertex_weights.append(influences)
        for triangle in mesh.loop_triangles:
            a, b, c = [vertex_offset + index + 1 for index in triangle.vertices]
            obj_file.write("f %d %d %d\n" % (a, b, c))
        vertex_offset += len(mesh.vertices)

original_frame = scene.frame_current
fps = scene.render.fps / scene.render.fps_base
frame_start = scene.frame_start
frame_end = scene.frame_end
frame_count = frame_end - frame_start + 1 if bones else 0
if frame_count > 6000:
    raise RuntimeError("The animation range exceeds 6000 sampled frames.")
basis = Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, -1, 0, 0), (0, 0, 0, 1)))
basis_inverse = basis.inverted()
with open(skin_path, "w", encoding="utf-8") as skin_file:
    skin_file.write("CSP_SKIN 1 %d %d %d %.9g\n" %
                    (vertex_offset, len(bones), frame_count, fps))
    skin_file.write("BONES\n")
    for bone in bones:
        parent_index = bone_indices[bone.parent.name] if bone.parent is not None else -1
        skin_file.write("%s %d\n" % (repr(bone.name), parent_index))
    skin_file.write("WEIGHTS\n")
    for influences in vertex_weights:
        skin_file.write(" ".join("%d %.9g" % item for item in influences) + "\n")
    skin_file.write("FRAMES\n")
    for frame in range(frame_start, frame_end + 1) if bones else []:
        scene.frame_set(frame)
        for bone in bones:
            pose_bone = armature.pose.bones.get(bone.name)
            if pose_bone is None:
                skin_matrix = Matrix.Identity(4)
            else:
                skin_matrix = (armature.matrix_world @ pose_bone.matrix
                               @ bone.matrix_local.inverted()
                               @ armature.matrix_world.inverted())
            engine_matrix = basis @ skin_matrix @ basis_inverse
            values = [engine_matrix[row][column] for row in range(4) for column in range(4)]
            skin_file.write(" ".join("%.9g" % value for value in values) + "\n")
scene.frame_set(original_frame)
)PY";
        script.close();

        std::wstring command = L"\"" + std::wstring(blender_path) + L"\" --background \""
            + blend_path.wstring() + L"\" --python \"" + script_path.wstring()
            + L"\" -- \"" + obj_path.wstring() + L"\" \"" + skin_path.wstring() + L"\"";
        STARTUPINFOW startup{};
        startup.cb = sizeof(startup);
        PROCESS_INFORMATION process{};
        if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE,
            CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process))
            return false;
        WaitForSingleObject(process.hProcess, INFINITE);
        DWORD exit_code = 1;
        GetExitCodeProcess(process.hProcess, &exit_code);
        CloseHandle(process.hThread);
        CloseHandle(process.hProcess);
        return exit_code == 0 && std::filesystem::exists(obj_path)
            && std::filesystem::exists(skin_path);
    }

    bool choose_blend_model()
    {
        wchar_t file_path[MAX_PATH]{};
        OPENFILENAMEW dialog{};
        dialog.lStructSize = sizeof(dialog);
        dialog.hwndOwner = parameter_window;
        dialog.lpstrFilter = L"Blender Project (*.blend)\0*.blend\0All Files (*.*)\0*.*\0";
        dialog.lpstrFile = file_path;
        dialog.nMaxFile = MAX_PATH;
        dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
        if (!GetOpenFileNameW(&dialog)) return false;
        std::filesystem::path obj_path;
        std::filesystem::path skin_path;
        if (!export_blend_to_obj(file_path, obj_path, skin_path)) return false;
        try
        {
            const csp::sphere_mesh mesh = csp::load_obj_mesh(obj_path);
            const csp::skeletal_animation animation =
                csp::load_skeletal_animation(skin_path, static_cast<std::uint32_t>(mesh.vertices.size()));
            if (mesh.vertices.empty() || mesh.indices.empty()) return false;
            if (!animation.bones.empty() && animation.frames.empty())
                throw std::runtime_error("Armature animation contains no sampled frames.");
        }
        catch (const std::exception& exception)
        {
            MessageBoxA(parameter_window, exception.what(), "Model import failed", MB_ICONERROR | MB_OK);
            return false;
        }
        active_obj_path = obj_path;
        active_skin_path = skin_path;
        custom_model_loaded = true;
        return true;
    }

    void relaunch_application()
    {
        wchar_t executable[MAX_PATH]{};
        GetModuleFileNameW(nullptr, executable, MAX_PATH);
        std::wstring command = L"\"" + std::wstring(executable) + L"\"";
        if (!active_obj_path.empty())
            command += L" --obj \"" + active_obj_path.wstring() + L"\"";
        if (!active_skin_path.empty())
            command += L" --skin \"" + active_skin_path.wstring() + L"\"";
        STARTUPINFOW startup{};
        startup.cb = sizeof(startup);
        PROCESS_INFORMATION process{};
        if (CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, 0,
            nullptr, nullptr, &startup, &process))
        {
            CloseHandle(process.hThread);
            CloseHandle(process.hProcess);
            PostQuitMessage(0);
        }
    }

    LRESULT CALLBACK parameter_window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
    {
        if (message == WM_CREATE)
        {
            for (int index = 0; index < parameter_count; ++index)
            {
                const int top = 18 + index * 62;
                CreateWindowExW(0, L"STATIC", parameter_descriptions[index].label,
                    WS_CHILD | WS_VISIBLE, 16, top, 245, 22, window, nullptr,
                    GetModuleHandleW(nullptr), nullptr);
                parameter_value_labels[index] = CreateWindowExW(0, L"STATIC", L"",
                    WS_CHILD | WS_VISIBLE | SS_RIGHT, 275, top, 245, 22, window, nullptr,
                    GetModuleHandleW(nullptr), nullptr);
                parameter_sliders[index] = CreateWindowExW(0, TRACKBAR_CLASSW, L"",
                    WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
                    16, top + 24, 504, 38, window,
                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_control_base + index)),
                    GetModuleHandleW(nullptr), nullptr);
                SendMessageW(parameter_sliders[index], TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
                SendMessageW(parameter_sliders[index], TBM_SETTICFREQ, 100, 0);
            }
            CreateWindowExW(0, L"BUTTON", L"\u30d1\u30e9\u30e1\u30fc\u30bf\u3092\u521d\u671f\u5316",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                16, 18 + parameter_count * 62, 240, 32, window,
                reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_reset_id)),
                GetModuleHandleW(nullptr), nullptr);
            pause_button = CreateWindowExW(0, L"BUTTON", L"\u30b7\u30df\u30e5\u30ec\u30fc\u30b7\u30e7\u30f3\u3092\u4e00\u6642\u505c\u6b62",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                280, 18 + parameter_count * 62, 240, 32, window,
                reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_pause_id)),
                GetModuleHandleW(nullptr), nullptr);
            performance_label = CreateWindowExW(0, L"STATIC", L"CPU -- ms    GPU -- ms",
                WS_CHILD | WS_VISIBLE, 16, 18 + parameter_count * 62 + 40,
                520, 68, window, nullptr, GetModuleHandleW(nullptr), nullptr);
            CreateWindowExW(0, L"BUTTON", L"Blender \u30e2\u30c7\u30eb\u3092\u8aad\u307f\u8fbc\u3080",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 16, 18 + parameter_count * 62 + 118,
                240, 32, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_model_id)),
                GetModuleHandleW(nullptr), nullptr);
            CreateWindowExW(0, L"BUTTON", L"\u30b7\u30df\u30e5\u30ec\u30fc\u30b7\u30e7\u30f3\u3092\u3084\u308a\u76f4\u3059",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 280, 18 + parameter_count * 62 + 118,
                240, 32, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_restart_id)),
                GetModuleHandleW(nullptr), nullptr);
            update_parameter_controls();
            update_performance_controls();
            parameter_content_height = 18 + parameter_count * 62 + 195;
            update_parameter_scrollbar(window);
            return 0;
        }
        if (message == WM_SIZE)
        {
            update_parameter_scrollbar(window);
            return 0;
        }
        if (message == WM_VSCROLL)
        {
            SCROLLINFO info{};
            info.cbSize = sizeof(info);
            info.fMask = SIF_ALL;
            GetScrollInfo(window, SB_VERT, &info);
            int position = info.nPos;
            switch (LOWORD(wparam))
            {
            case SB_LINEUP: position -= 40; break;
            case SB_LINEDOWN: position += 40; break;
            case SB_PAGEUP: position -= static_cast<int>(info.nPage); break;
            case SB_PAGEDOWN: position += static_cast<int>(info.nPage); break;
            case SB_THUMBTRACK: position = info.nTrackPos; break;
            case SB_THUMBPOSITION: position = info.nTrackPos; break;
            case SB_TOP: position = 0; break;
            case SB_BOTTOM: position = parameter_content_height; break;
            default: return 0;
            }
            scroll_parameter_window(window, position);
            return 0;
        }
        if (message == WM_MOUSEWHEEL)
        {
            const int wheel_delta = GET_WHEEL_DELTA_WPARAM(wparam);
            scroll_parameter_window(window, parameter_scroll_position
                - (wheel_delta / WHEEL_DELTA) * 60);
            return 0;
        }
        if (message == WM_HSCROLL)
        {
            apply_parameter_control(reinterpret_cast<HWND>(lparam));
            return 0;
        }
        if (message == WM_COMMAND && LOWORD(wparam) == parameter_reset_id)
        {
            if (active_renderer != nullptr)
                active_renderer->set_simulation_parameters(csp::simulation_parameters{});
            update_parameter_controls();
            return 0;
        }
        if (message == WM_COMMAND && LOWORD(wparam) == parameter_model_id)
        {
            if (choose_blend_model()) relaunch_application();
            return 0;
        }
        if (message == WM_COMMAND && LOWORD(wparam) == parameter_restart_id)
        {
            relaunch_application();
            return 0;
        }
        if (message == WM_COMMAND && LOWORD(wparam) == parameter_pause_id)
        {
            if (active_renderer != nullptr)
            {
                active_renderer->set_simulation_paused(!active_renderer->is_simulation_paused());
                SetWindowTextW(pause_button, active_renderer->is_simulation_paused()
                    ? L"\u30b7\u30df\u30e5\u30ec\u30fc\u30b7\u30e7\u30f3\u3092\u518d\u958b" : L"\u30b7\u30df\u30e5\u30ec\u30fc\u30b7\u30e7\u30f3\u3092\u4e00\u6642\u505c\u6b62");
            }
            return 0;
        }
        if (message == WM_CLOSE)
        {
            ShowWindow(window, SW_HIDE);
            return 0;
        }
        return DefWindowProcW(window, message, wparam, lparam);
    }

    void update_window_title(HWND window)
    {
        if (active_renderer == nullptr) return;
        const wchar_t* mode_name = active_renderer->get_solver_mode() == csp::solver_mode::camera_surface
            ? L"\u8868\u9762\u7c92\u5b50 XPBD"
            : L"\u56db\u9762\u4f53 XPBD";
        const std::wstring title = std::wstring(L"CameraSkinPhysics | ") + mode_name
            + (custom_model_loaded
                ? L" | F2: \u8868\u9762\u7c92\u5b50  F3: \u30d1\u30e9\u30e1\u30fc\u30bf | \u5916\u90e8\u30e2\u30c7\u30eb"
                : L" | F1: \u56db\u9762\u4f53 XPBD  F2: \u8868\u9762\u7c92\u5b50  F3: \u30d1\u30e9\u30e1\u30fc\u30bf | output/ \u306b\u8a08\u6e2c\u7d50\u679c");
        SetWindowTextW(window, title.c_str());
    }

    std::uint32_t parse_benchmark_frame_count(PCWSTR command_line)
    {
        if (command_line == nullptr) return 0;
        const std::wstring arguments(command_line);
        const std::size_t flag_position = arguments.find(L"--benchmark");
        if (flag_position == std::wstring::npos) return 0;

        constexpr std::uint32_t default_frame_count = 300;
        const std::size_t value_position = flag_position + 11;
        if (value_position >= arguments.size() || arguments[value_position] != L'=')
            return default_frame_count;

        wchar_t* end = nullptr;
        const unsigned long requested =
            std::wcstoul(arguments.c_str() + value_position + 1, &end, 10);
        if (end == arguments.c_str() + value_position + 1 || requested == 0 || requested > 36000)
            return default_frame_count;
        return static_cast<std::uint32_t>(requested);
    }

    LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
    {
        if (message == WM_KEYDOWN && wparam == VK_ESCAPE)
        {
            PostQuitMessage(0);
            return 0;
        }
        if (message == WM_KEYDOWN && benchmark_running
            && (wparam == VK_F1 || wparam == VK_F2))
            return 0;

        if (message == WM_KEYDOWN && active_renderer != nullptr)
        {
            if (wparam == VK_F1 && !custom_model_loaded)
            {
                active_renderer->set_solver_mode(csp::solver_mode::tetrahedral_xpbd);
                update_window_title(window);
                return 0;
            }
            if (wparam == VK_F2)
            {
                active_renderer->set_solver_mode(csp::solver_mode::camera_surface);
                update_window_title(window);
                return 0;
            }
            if (wparam == VK_F3 && parameter_window != nullptr)
            {
                ShowWindow(parameter_window,
                    IsWindowVisible(parameter_window) ? SW_HIDE : SW_SHOW);
                return 0;
            }
        }
        if (message == WM_DESTROY)
        {
            PostQuitMessage(0);
            return 0;
        }
        return DefWindowProcW(window, message, wparam, lparam);
    }
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR command_line, int show_command)
{
    INITCOMMONCONTROLSEX common_controls{};
    common_controls.dwSize = sizeof(common_controls);
    common_controls.dwICC = ICC_BAR_CLASSES;
    InitCommonControlsEx(&common_controls);

    WNDCLASSEXW parameter_class{};
    parameter_class.cbSize = sizeof(parameter_class);
    parameter_class.lpfnWndProc = parameter_window_proc;
    parameter_class.hInstance = instance;
    parameter_class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    parameter_class.lpszClassName = parameter_window_class_name;
    if (RegisterClassExW(&parameter_class) == 0) return -1;

    WNDCLASSEXW window_class{};
    window_class.cbSize = sizeof(window_class);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = window_proc;
    window_class.hInstance = instance;
    window_class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    window_class.lpszClassName = window_class_name;
    if (RegisterClassExW(&window_class) == 0) return -1;

    RECT window_rect{0, 0, 1440, 900};
    const DWORD window_style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&window_rect, window_style, FALSE);
    HWND window = CreateWindowExW(0, window_class_name,
        L"CameraSkinPhysics - GPU\u30bd\u30d5\u30c8\u30dc\u30c7\u30a3\u6bd4\u8f03",
        window_style, CW_USEDEFAULT, CW_USEDEFAULT,
        window_rect.right - window_rect.left, window_rect.bottom - window_rect.top,
        nullptr, nullptr, instance, nullptr);
    if (window == nullptr) return -1;

    try
    {
        int argument_count = 0;
        LPWSTR* arguments = CommandLineToArgvW(GetCommandLineW(), &argument_count);
        std::filesystem::path startup_obj_path;
        std::filesystem::path startup_skin_path;
        if (arguments != nullptr)
        {
            for (int argument = 1; argument + 1 < argument_count; ++argument)
            {
                if (std::wstring(arguments[argument]) == L"--obj")
                    startup_obj_path = arguments[argument + 1];
                else if (std::wstring(arguments[argument]) == L"--skin")
                    startup_skin_path = arguments[argument + 1];
            }
            LocalFree(arguments);
        }
        custom_model_loaded = !startup_obj_path.empty();
        active_obj_path = startup_obj_path;
        active_skin_path = startup_skin_path;
        const csp::sphere_mesh surface_mesh = custom_model_loaded
            ? csp::load_obj_mesh(startup_obj_path) : csp::make_sphere_mesh();
        csp::skeletal_animation animation{};
        if (!startup_skin_path.empty())
            animation = csp::load_skeletal_animation(startup_skin_path,
                static_cast<std::uint32_t>(surface_mesh.vertices.size()));
        if (!custom_model_loaded && (surface_mesh.vertices.size() != 100000 || surface_mesh.indices.size() != 599988
            || surface_mesh.edges.size() != 299994
            || surface_mesh.edge_incidence_ranges.size() != surface_mesh.vertices.size()
            || surface_mesh.incident_corrections.size() != 599988))
        {
            MessageBoxW(window, L"\u7403\u4f53\u8868\u9762\u306e\u30c8\u30dd\u30ed\u30b8\u30fc\u691c\u8a3c\u306b\u5931\u6557\u3057\u307e\u3057\u305f\u3002", L"CameraSkinPhysics",
                MB_ICONERROR | MB_OK);
            return -1;
        }

        const csp::tetrahedral_mesh volume_mesh = custom_model_loaded
            ? make_surface_only_volume(surface_mesh) : csp::make_tetrahedral_mesh(surface_mesh);
        RECT client_rect{};
        GetClientRect(window, &client_rect);
        csp::d3d12_renderer renderer;
        if (!renderer.initialize(window,
            static_cast<std::uint32_t>(client_rect.right - client_rect.left),
            static_cast<std::uint32_t>(client_rect.bottom - client_rect.top),
            surface_mesh, volume_mesh, startup_skin_path.empty() ? nullptr : &animation))
        {
            MessageBoxW(window,
                L"Direct3D 12\u306e\u521d\u671f\u5316\u306b\u5931\u6557\u3057\u307e\u3057\u305f\u3002GPU\u30c9\u30e9\u30a4\u30d0\u30fc\u3068DXC\u306b\u3088\u308b\u30b7\u30a7\u30fc\u30c0\u30fc\u751f\u6210\u3092\u78ba\u8a8d\u3057\u3066\u304f\u3060\u3055\u3044\u3002",
                L"CameraSkinPhysics", MB_ICONERROR | MB_OK);
            return -1;
        }

        if (custom_model_loaded) renderer.set_solver_mode(csp::solver_mode::camera_surface);
        active_renderer = &renderer;
        update_window_title(window);
        parameter_window = CreateWindowExW(WS_EX_TOOLWINDOW,
            parameter_window_class_name, L"CameraSkinPhysics | \u30d1\u30e9\u30e1\u30fc\u30bf",
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_VSCROLL | WS_CLIPCHILDREN,
            CW_USEDEFAULT, CW_USEDEFAULT, 560, 700,
            window, nullptr, instance, nullptr);
        if (parameter_window == nullptr)
            throw std::runtime_error("Failed to create parameter window.");
        ShowWindow(parameter_window, SW_SHOW);
        ShowWindow(window, show_command);

        const std::uint32_t benchmark_frame_count = custom_model_loaded ? 0u : parse_benchmark_frame_count(command_line);
        if (benchmark_frame_count > 0)
        {
            benchmark_running = true;
            const csp::solver_mode benchmark_modes[2] = {
                csp::solver_mode::tetrahedral_xpbd,
                csp::solver_mode::camera_surface
            };
            MSG benchmark_message{};
            bool continue_benchmark = true;
            for (const csp::solver_mode mode : benchmark_modes)
            {
                if (!continue_benchmark) break;
                renderer.set_solver_mode(mode);
                update_window_title(window);
                for (std::uint32_t frame = 0; frame < benchmark_frame_count; ++frame)
                {
                    while (PeekMessageW(&benchmark_message, nullptr, 0, 0, PM_REMOVE))
                    {
                        if (benchmark_message.message == WM_QUIT)
                        {
                            continue_benchmark = false;
                            break;
                        }
                        TranslateMessage(&benchmark_message);
                        DispatchMessageW(&benchmark_message);
                    }
                    if (!continue_benchmark) break;
                    renderer.render(1.0f / 60.0f);
                }
            }
            benchmark_running = false;
            active_renderer = nullptr;
            renderer.wait_gpu();
            return 0;
        }

        MSG message{};
        std::chrono::steady_clock::time_point previous_frame = std::chrono::steady_clock::now();
        while (message.message != WM_QUIT)
        {
            if (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
            {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }
            else
            {
                const std::chrono::steady_clock::time_point current_frame = std::chrono::steady_clock::now();
                const float delta_time = std::chrono::duration<float>(current_frame - previous_frame).count();
                previous_frame = current_frame;
                renderer.render(delta_time);
                update_performance_controls();
            }
        }

        active_renderer = nullptr;
        renderer.wait_gpu();
        return static_cast<int>(message.wParam);
    }
    catch (const std::exception& exception)
    {
        benchmark_running = false;
        active_renderer = nullptr;
        const std::string message_text = exception.what();
        const int required = MultiByteToWideChar(CP_UTF8, 0, message_text.c_str(), -1, nullptr, 0);
        std::wstring wide_message(static_cast<std::size_t>(std::max(required, 1)), L'\0');
        if (required > 0)
            MultiByteToWideChar(CP_UTF8, 0, message_text.c_str(), -1, wide_message.data(), required);
        MessageBoxW(window, wide_message.c_str(), L"CameraSkinPhysics \u30a8\u30e9\u30fc", MB_ICONERROR | MB_OK);
        return -1;
    }
}
