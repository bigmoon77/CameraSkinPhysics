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
    bool custom_model_loaded = false;

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

    bool export_blend_to_obj(const std::filesystem::path& blend_path, std::filesystem::path& obj_path)
    {
        wchar_t blender_path[MAX_PATH]{};
        if (SearchPathW(nullptr, L"blender.exe", nullptr, MAX_PATH, blender_path, nullptr) == 0)
        {
            MessageBoxW(parameter_window, L"blender.exe が PATH に見つかりません。Blender の bin フォルダーを PATH に追加してください。",
                L"モデル読み込み", MB_ICONERROR | MB_OK);
            return false;
        }

        wchar_t temporary_directory[MAX_PATH]{};
        if (GetTempPathW(MAX_PATH, temporary_directory) == 0) return false;
        const std::filesystem::path script_path = std::filesystem::path(temporary_directory) / L"csp_blend_export.py";
        obj_path = std::filesystem::path(temporary_directory) / L"csp_blend_import.obj";
        std::ofstream script(script_path, std::ios::binary);
        if (!script) return false;
        script << R"PY(import bpy
import sys
from mathutils import Vector
args = sys.argv[sys.argv.index("--") + 1:]
output_path = args[0]
depsgraph = bpy.context.evaluated_depsgraph_get()
vertex_offset = 0
with open(output_path, "w", encoding="utf-8") as output:
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        matrix = evaluated.matrix_world
        for vertex in mesh.vertices:
            p = matrix @ vertex.co
            output.write("v %.9g %.9g %.9g\n" % (p.x, p.z, -p.y))
        for triangle in mesh.loop_triangles:
            a, b, c = [vertex_offset + index + 1 for index in triangle.vertices]
            output.write("f %d %d %d\n" % (a, b, c))
        vertex_offset += len(mesh.vertices)
        evaluated.to_mesh_clear()
)PY";
        script.close();

        std::wstring command = L"\"" + std::wstring(blender_path) + L"\" --background \""
            + blend_path.wstring() + L"\" --python \"" + script_path.wstring()
            + L"\" -- \"" + obj_path.wstring() + L"\"";
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
        return exit_code == 0 && std::filesystem::exists(obj_path);
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
        if (!export_blend_to_obj(file_path, obj_path)) return false;
        try
        {
            csp::sphere_mesh mesh = csp::load_obj_mesh(obj_path);
            if (mesh.vertices.empty() || mesh.indices.empty()) return false;
        }
        catch (const std::exception& exception)
        {
            MessageBoxA(parameter_window, exception.what(), "Model import failed", MB_ICONERROR | MB_OK);
            return false;
        }
        active_obj_path = obj_path;
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
            CreateWindowExW(0, L"BUTTON", L"Blender モデルを読み込む",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 16, 18 + parameter_count * 62 + 118,
                240, 32, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_model_id)),
                GetModuleHandleW(nullptr), nullptr);
            CreateWindowExW(0, L"BUTTON", L"シミュレーションをやり直す",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 280, 18 + parameter_count * 62 + 118,
                240, 32, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_restart_id)),
                GetModuleHandleW(nullptr), nullptr);
            update_parameter_controls();
            update_performance_controls();
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
            + L" | F1: \u56db\u9762\u4f53 XPBD  F2: \u8868\u9762\u7c92\u5b50  F3: \u30d1\u30e9\u30e1\u30fc\u30bf | output/ \u306b\u8a08\u6e2c\u7d50\u679c";
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
        if (arguments != nullptr)
        {
            for (int argument = 1; argument + 1 < argument_count; ++argument)
                if (std::wstring(arguments[argument]) == L"--obj") startup_obj_path = arguments[argument + 1];
            LocalFree(arguments);
        }
        custom_model_loaded = !startup_obj_path.empty();
        active_obj_path = startup_obj_path;
        const csp::sphere_mesh surface_mesh = custom_model_loaded
            ? csp::load_obj_mesh(startup_obj_path) : csp::make_sphere_mesh();
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
            surface_mesh, volume_mesh))
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
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
            CW_USEDEFAULT, CW_USEDEFAULT, 560, 18 + parameter_count * 62 + 155,
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
