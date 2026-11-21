#include "d3d12_renderer.hpp"
#include "sphere_mesh.hpp"
#include "tetrahedral_mesh.hpp"

#include <algorithm>
#include <chrono>
#include <cwchar>
#include <exception>
#include <string>
#include <windows.h>

namespace
{
    constexpr wchar_t window_class_name[] = L"CameraSkinPhysicsWindow";
    csp::d3d12_renderer* active_renderer = nullptr;

    void update_window_title(HWND window)
    {
        if (active_renderer == nullptr) return;
        const wchar_t* mode_name = active_renderer->get_solver_mode() == csp::solver_mode::camera_surface
            ? L"Camera surface particles"
            : L"Tetrahedral XPBD";
        const std::wstring title = std::wstring(L"CameraSkinPhysics | ") + mode_name
            + L" | F1: XPBD  F2: Camera surface | benchmark in output/";
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
            || surface_mesh.incident_edges.size() != 599988)
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
        ShowWindow(window, show_command);

        const std::uint32_t benchmark_frame_count = parse_benchmark_frame_count(command_line);
        if (benchmark_frame_count > 0)
        {
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
            }
        }

        active_renderer = nullptr;
        renderer.wait_gpu();
        return static_cast<int>(message.wParam);
    }
    catch (const std::exception& exception)
    {
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
