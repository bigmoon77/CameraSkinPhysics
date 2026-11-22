#include "d3d12_renderer.hpp"
#include "sphere_mesh.hpp"
#include "tetrahedral_mesh.hpp"

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

namespace
{
    constexpr wchar_t window_class_name[] = L"CameraSkinPhysicsWindow";
    constexpr wchar_t parameter_window_class_name[] = L"CameraSkinPhysicsParameters";
    constexpr int parameter_control_base = 100;
    constexpr int parameter_reset_id = 200;
    constexpr int parameter_count = 8;
    HWND parameter_sliders[parameter_count]{};
    HWND parameter_value_labels[parameter_count]{};
    HWND parameter_window = nullptr;
    HWND performance_label = nullptr;
    csp::d3d12_renderer* active_renderer = nullptr;
    bool benchmark_running = false;

    struct parameter_description
    {
        const wchar_t* label;
        float minimum;
        float maximum;
        float scale;
        bool integer;
    };

    constexpr parameter_description parameter_descriptions[parameter_count] = {
        {L"Solver iterations", 1.0f, 4.0f, 1.0f, true},
        {L"Edge relaxation", 0.0f, 1.5f, 1000.0f, false},
        {L"Max correction", 0.0001f, 0.05f, 100000.0f, false},
        {L"Particle radius", 0.001f, 0.05f, 100000.0f, false},
        {L"Pushout", 0.0f, 0.03f, 100000.0f, false},
        {L"Velocity friction", 0.0f, 1.0f, 1000.0f, false},
        {L"Restitution", 0.0f, 1.0f, 1000.0f, false},
        {L"Contact band", 0.005f, 0.5f, 1000.0f, false}
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
            const int position = static_cast<int>(std::lround(
                (value - description.minimum) / (description.maximum - description.minimum) * 1000.0f));
            SendMessageW(parameter_sliders[index], TBM_SETPOS, TRUE, position);
            std::wostringstream text;
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
            << L"CPU " << metrics.cpu_frame_ms << L" ms    GPU " << metrics.gpu_total_ms << L" ms\n"
            << L"Simulation " << metrics.gpu_simulation_ms << L" ms    Collision "
            << metrics.gpu_collision_ms << L" ms    Render " << metrics.gpu_render_ms << L" ms\n"
            << L"Edge Solve " << metrics.camera_edge_solve_ms << L" ms    Gather "
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
        const float value = description.integer
            ? std::round(description.minimum + normalized * (description.maximum - description.minimum))
            : description.minimum + normalized * (description.maximum - description.minimum);
        csp::simulation_parameters parameters = active_renderer->get_simulation_parameters();
        set_parameter_value(parameters, index, value);
        active_renderer->set_simulation_parameters(parameters);
        update_parameter_controls();
    }

    LRESULT CALLBACK parameter_window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
    {
        if (message == WM_CREATE)
        {
            for (int index = 0; index < parameter_count; ++index)
            {
                const int top = 18 + index * 76;
                CreateWindowExW(0, L"STATIC", parameter_descriptions[index].label,
                    WS_CHILD | WS_VISIBLE, 16, top, 210, 22, window, nullptr,
                    GetModuleHandleW(nullptr), nullptr);
                parameter_value_labels[index] = CreateWindowExW(0, L"STATIC", L"",
                    WS_CHILD | WS_VISIBLE | SS_RIGHT, 250, top, 150, 22, window, nullptr,
                    GetModuleHandleW(nullptr), nullptr);
                parameter_sliders[index] = CreateWindowExW(0, TRACKBAR_CLASSW, L"",
                    WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
                    16, top + 24, 384, 38, window,
                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_control_base + index)),
                    GetModuleHandleW(nullptr), nullptr);
                SendMessageW(parameter_sliders[index], TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
                SendMessageW(parameter_sliders[index], TBM_SETTICFREQ, 100, 0);
            }
            CreateWindowExW(0, L"BUTTON", L"Reset parameters",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                16, 18 + parameter_count * 76, 384, 32, window,
                reinterpret_cast<HMENU>(static_cast<INT_PTR>(parameter_reset_id)),
                GetModuleHandleW(nullptr), nullptr);
            performance_label = CreateWindowExW(0, L"STATIC", L"CPU -- ms    GPU -- ms",
                WS_CHILD | WS_VISIBLE, 16, 18 + parameter_count * 76 + 40,
                400, 58, window, nullptr, GetModuleHandleW(nullptr), nullptr);
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
            ? L"Camera surface particles"
            : L"Tetrahedral XPBD";
        const std::wstring title = std::wstring(L"CameraSkinPhysics | ") + mode_name
            + L" | F1: XPBD  F2: Camera surface  F3: Parameters | benchmark in output/";
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
            if (wparam == VK_F1)
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
        L"CameraSkinPhysics - GPU Soft Body Comparison",
        window_style, CW_USEDEFAULT, CW_USEDEFAULT,
        window_rect.right - window_rect.left, window_rect.bottom - window_rect.top,
        nullptr, nullptr, instance, nullptr);
    if (window == nullptr) return -1;

    try
    {
        const csp::sphere_mesh surface_mesh = csp::make_sphere_mesh();
        if (surface_mesh.vertices.size() != 100000 || surface_mesh.indices.size() != 599988
            || surface_mesh.edges.size() != 299994
            || surface_mesh.edge_incidence_ranges.size() != surface_mesh.vertices.size()
            || surface_mesh.incident_corrections.size() != 599988)
        {
            MessageBoxW(window, L"Sphere surface topology validation failed.", L"CameraSkinPhysics",
                MB_ICONERROR | MB_OK);
            return -1;
        }

        const csp::tetrahedral_mesh volume_mesh = csp::make_tetrahedral_mesh(surface_mesh);
        RECT client_rect{};
        GetClientRect(window, &client_rect);
        csp::d3d12_renderer renderer;
        if (!renderer.initialize(window,
            static_cast<std::uint32_t>(client_rect.right - client_rect.left),
            static_cast<std::uint32_t>(client_rect.bottom - client_rect.top),
            surface_mesh, volume_mesh))
        {
            MessageBoxW(window,
                L"Direct3D 12 initialization failed. Check the GPU driver and DXC shader build.",
                L"CameraSkinPhysics", MB_ICONERROR | MB_OK);
            return -1;
        }

        active_renderer = &renderer;
        update_window_title(window);
        parameter_window = CreateWindowExW(WS_EX_TOOLWINDOW,
            parameter_window_class_name, L"CameraSkinPhysics | Live Parameters",
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
            CW_USEDEFAULT, CW_USEDEFAULT, 440, 18 + parameter_count * 76 + 105,
            window, nullptr, instance, nullptr);
        if (parameter_window == nullptr)
            throw std::runtime_error("Failed to create parameter window.");
        ShowWindow(parameter_window, SW_SHOW);
        ShowWindow(window, show_command);

        const std::uint32_t benchmark_frame_count = parse_benchmark_frame_count(command_line);
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
        MessageBoxW(window, wide_message.c_str(), L"CameraSkinPhysics error", MB_ICONERROR | MB_OK);
        return -1;
    }
}
