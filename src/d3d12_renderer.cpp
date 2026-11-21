#include "d3d12_renderer.hpp"

#include <stdexcept>

namespace csp
{
    d3d12_renderer::~d3d12_renderer()
    {
        wait_gpu();
        if (fence_event_ != nullptr) CloseHandle(fence_event_);
    }

    bool d3d12_renderer::initialize(HWND window, std::uint32_t width, std::uint32_t height)
    {
        UINT factory_flags = 0;
#if defined(_DEBUG)
        Microsoft::WRL::ComPtr<ID3D12Debug> debug_controller;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug_controller))))
        {
            debug_controller->EnableDebugLayer();
            factory_flags |= DXGI_CREATE_FACTORY_DEBUG;
        }
#endif
        if (FAILED(CreateDXGIFactory2(factory_flags, IID_PPV_ARGS(&factory_)))) return false;

        Microsoft::WRL::ComPtr<IDXGIAdapter1> adapter;
        for (UINT index = 0; factory_->EnumAdapters1(index, &adapter) != DXGI_ERROR_NOT_FOUND; ++index)
        {
            DXGI_ADAPTER_DESC1 description{};
            adapter->GetDesc1(&description);
            if ((description.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0) continue;
            if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&device_)))) break;
            adapter.Reset();
        }
        if (!device_ && FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&device_)))) return false;

        D3D12_COMMAND_QUEUE_DESC queue_description{};
        queue_description.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if (FAILED(device_->CreateCommandQueue(&queue_description, IID_PPV_ARGS(&command_queue_)))) return false;

        DXGI_SWAP_CHAIN_DESC1 swap_description{};
        swap_description.Width = width;
        swap_description.Height = height;
        swap_description.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swap_description.SampleDesc.Count = 1;
        swap_description.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swap_description.BufferCount = frame_count;
        swap_description.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;

        Microsoft::WRL::ComPtr<IDXGISwapChain1> swap_chain;
        if (FAILED(factory_->CreateSwapChainForHwnd(command_queue_.Get(), window, &swap_description,
            nullptr, nullptr, &swap_chain))) return false;
        if (FAILED(swap_chain.As(&swap_chain_))) return false;
        factory_->MakeWindowAssociation(window, DXGI_MWA_NO_ALT_ENTER);

        D3D12_DESCRIPTOR_HEAP_DESC heap_description{};
        heap_description.NumDescriptors = frame_count;
        heap_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        if (FAILED(device_->CreateDescriptorHeap(&heap_description, IID_PPV_ARGS(&rtv_heap_)))) return false;
        rtv_descriptor_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        for (std::uint32_t index = 0; index < frame_count; ++index)
        {
            if (FAILED(device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                IID_PPV_ARGS(&command_allocators_[index])))) return false;
        }
        if (FAILED(device_->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT,
            command_allocators_[0].Get(), nullptr, IID_PPV_ARGS(&command_list_)))) return false;
        if (FAILED(command_list_->Close())) return false;

        if (FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_)))) return false;
        fence_event_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if (fence_event_ == nullptr) return false;

        create_render_targets();
        frame_index_ = swap_chain_->GetCurrentBackBufferIndex();
        return true;
    }

    void d3d12_renderer::create_render_targets()
    {
        D3D12_CPU_DESCRIPTOR_HANDLE handle = rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        for (std::uint32_t index = 0; index < frame_count; ++index)
        {
            if (FAILED(swap_chain_->GetBuffer(index, IID_PPV_ARGS(&render_targets_[index]))))
                throw std::runtime_error("Failed to acquire swap-chain buffer.");
            device_->CreateRenderTargetView(render_targets_[index].Get(), nullptr, handle);
            handle.ptr += rtv_descriptor_size_;
        }
    }

    void d3d12_renderer::render()
    {
        ID3D12CommandAllocator* allocator = command_allocators_[frame_index_].Get();
        if (FAILED(allocator->Reset())) throw std::runtime_error("Command allocator reset failed.");
        if (FAILED(command_list_->Reset(allocator, nullptr))) throw std::runtime_error("Command list reset failed.");

        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource = render_targets_[frame_index_].Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        command_list_->ResourceBarrier(1, &barrier);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += static_cast<SIZE_T>(frame_index_) * rtv_descriptor_size_;
        command_list_->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        constexpr float clear_color[4] = {0.025f, 0.035f, 0.055f, 1.0f};
        command_list_->ClearRenderTargetView(rtv, clear_color, 0, nullptr);

        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        command_list_->ResourceBarrier(1, &barrier);
        if (FAILED(command_list_->Close())) throw std::runtime_error("Command list close failed.");

        ID3D12CommandList* command_lists[] = {command_list_.Get()};
        command_queue_->ExecuteCommandLists(1, command_lists);
        if (FAILED(swap_chain_->Present(1, 0))) throw std::runtime_error("Swap-chain present failed.");

        const std::uint64_t signal_value = next_fence_value_++;
        if (FAILED(command_queue_->Signal(fence_.Get(), signal_value))) throw std::runtime_error("Queue signal failed.");
        frame_fence_values_[frame_index_] = signal_value;
        frame_index_ = swap_chain_->GetCurrentBackBufferIndex();

        if (fence_->GetCompletedValue() < frame_fence_values_[frame_index_])
        {
            if (FAILED(fence_->SetEventOnCompletion(frame_fence_values_[frame_index_], fence_event_)))
                throw std::runtime_error("Fence wait setup failed.");
            WaitForSingleObject(fence_event_, INFINITE);
        }
    }

    void d3d12_renderer::wait_gpu()
    {
        if (!command_queue_ || !fence_ || fence_event_ == nullptr) return;
        const std::uint64_t signal_value = next_fence_value_++;
        if (SUCCEEDED(command_queue_->Signal(fence_.Get(), signal_value))
            && fence_->GetCompletedValue() < signal_value)
        {
            if (SUCCEEDED(fence_->SetEventOnCompletion(signal_value, fence_event_)))
                WaitForSingleObject(fence_event_, INFINITE);
        }
    }
}
