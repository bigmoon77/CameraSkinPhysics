#pragma once
#include <array>
#include <cstdint>
#include <wrl.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <windows.h>

namespace csp
{
    class d3d12_renderer
    {
    public:
        d3d12_renderer() = default;
        ~d3d12_renderer();
        d3d12_renderer(const d3d12_renderer&) = delete;
        d3d12_renderer& operator=(const d3d12_renderer&) = delete;

        bool initialize(HWND window, std::uint32_t width, std::uint32_t height);
        void render();
        void wait_gpu();

    private:
        static constexpr std::uint32_t frame_count = 2;
        void create_render_targets();

        Microsoft::WRL::ComPtr<IDXGIFactory6> factory_;
        Microsoft::WRL::ComPtr<ID3D12Device> device_;
        Microsoft::WRL::ComPtr<ID3D12CommandQueue> command_queue_;
        Microsoft::WRL::ComPtr<IDXGISwapChain3> swap_chain_;
        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtv_heap_;
        Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> command_list_;
        Microsoft::WRL::ComPtr<ID3D12Fence> fence_;
        std::array<Microsoft::WRL::ComPtr<ID3D12CommandAllocator>, frame_count> command_allocators_;
        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, frame_count> render_targets_;
        std::array<std::uint64_t, frame_count> frame_fence_values_{};
        std::uint32_t rtv_descriptor_size_ = 0;
        std::uint32_t frame_index_ = 0;
        std::uint64_t next_fence_value_ = 1;
        HANDLE fence_event_ = nullptr;
    };
}
