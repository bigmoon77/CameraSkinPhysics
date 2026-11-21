#include "d3d12_renderer.hpp"
#include "sphere_mesh.hpp"

#include <chrono>
#include <exception>
#include <windows.h>

namespace
{
    constexpr wchar_t window_class_name[] = L"CameraSkinPhysicsWindow";

    LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
    {
        if (message == WM_DESTROY)
        {
            PostQuitMessage(0);
            return 0;
        }
        return DefWindowProcW(window, message, wparam, lparam);
    }
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show_command)
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
    AdjustWindowRect(&window_rect, WS_OVERLAPPEDWINDOW, FALSE);
    HWND window = CreateWindowExW(0, window_class_name, L"CameraSkinPhysics - Direct3D 12",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
        window_rect.right - window_rect.left, window_rect.bottom - window_rect.top,
        nullptr, nullptr, instance, nullptr);
    if (window == nullptr) return -1;

    RECT client_rect{};
    GetClientRect(window, &client_rect);
    csp::d3d12_renderer renderer;
    if (!renderer.initialize(window,
        static_cast<std::uint32_t>(client_rect.right - client_rect.left),
        static_cast<std::uint32_t>(client_rect.bottom - client_rect.top)))
    {
        MessageBoxW(window, L"Direct3D 12 initialization failed.", L"CameraSkinPhysics", MB_ICONERROR | MB_OK);
        return -1;
    }

    const csp::sphere_mesh reference_mesh = csp::make_sphere_mesh();
    if (reference_mesh.vertices.size() != 100000)
    {
        MessageBoxW(window, L"Sphere vertex count validation failed.", L"CameraSkinPhysics", MB_ICONERROR | MB_OK);
        return -1;
    }

    ShowWindow(window, show_command);
    MSG message{};
    while (message.message != WM_QUIT)
    {
        if (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        else
        {
            renderer.render();
        }
    }
    renderer.wait_gpu();
    return static_cast<int>(message.wParam);
}
