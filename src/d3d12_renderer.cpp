#include "d3d12_renderer.hpp"

#include <DirectXMath.h>
#include <d3d12sdklayers.h>
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace csp
{
    namespace
    {
        std::vector<char> read_binary_file(const std::filesystem::path& path)
        {
            std::ifstream stream(path, std::ios::binary);
            if (!stream) return {};
            return std::vector<char>(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
        }

        D3D12_HEAP_PROPERTIES make_heap_properties(D3D12_HEAP_TYPE type)
        {
            D3D12_HEAP_PROPERTIES properties{};
            properties.Type = type;
            properties.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
            properties.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
            properties.CreationNodeMask = 1;
            properties.VisibleNodeMask = 1;
            return properties;
        }

        D3D12_RESOURCE_DESC make_buffer_desc(UINT64 size)
        {
            D3D12_RESOURCE_DESC description{};
            description.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            description.Width = size;
            description.Height = 1;
            description.DepthOrArraySize = 1;
            description.MipLevels = 1;
            description.SampleDesc.Count = 1;
            description.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
            return description;
        }

        struct draw_constants
        {
            DirectX::XMFLOAT4X4 view_projection;
            DirectX::XMFLOAT4X4 world;
            DirectX::XMFLOAT4 color;
        };
        static_assert(sizeof(draw_constants) == sizeof(float) * 36);
    }

    d3d12_renderer::~d3d12_renderer()
    {
        wait_gpu();
        if (fence_event_ != nullptr) CloseHandle(fence_event_);
    }

    bool d3d12_renderer::initialize(HWND window, std::uint32_t width, std::uint32_t height, const sphere_mesh& mesh)
    {
        width_ = width;
        height_ = height;
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
            if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device_)))) break;
            adapter.Reset();
        }
        if (!device_ && FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device_)))) return false;

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

        D3D12_DESCRIPTOR_HEAP_DESC rtv_description{};
        rtv_description.NumDescriptors = frame_count;
        rtv_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        if (FAILED(device_->CreateDescriptorHeap(&rtv_description, IID_PPV_ARGS(&rtv_heap_)))) return false;
        rtv_descriptor_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        D3D12_DESCRIPTOR_HEAP_DESC dsv_description{};
        dsv_description.NumDescriptors = 1;
        dsv_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        if (FAILED(device_->CreateDescriptorHeap(&dsv_description, IID_PPV_ARGS(&dsv_heap_)))) return false;

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
        if (!create_depth_buffer() || !create_sphere_buffers(mesh) || !create_sphere_pipeline()) return false;
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

    bool d3d12_renderer::create_depth_buffer()
    {
        D3D12_RESOURCE_DESC description{};
        description.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        description.Width = width_;
        description.Height = height_;
        description.DepthOrArraySize = 1;
        description.MipLevels = 1;
        description.Format = DXGI_FORMAT_D32_FLOAT;
        description.SampleDesc.Count = 1;
        description.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        description.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clear_value{};
        clear_value.Format = DXGI_FORMAT_D32_FLOAT;
        clear_value.DepthStencil.Depth = 1.0f;
        clear_value.DepthStencil.Stencil = 0;
        const D3D12_HEAP_PROPERTIES heap_properties = make_heap_properties(D3D12_HEAP_TYPE_DEFAULT);
        if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE, &description,
            D3D12_RESOURCE_STATE_DEPTH_WRITE, &clear_value, IID_PPV_ARGS(&depth_buffer_)))) return false;
        device_->CreateDepthStencilView(depth_buffer_.Get(), nullptr, dsv_heap_->GetCPUDescriptorHandleForHeapStart());
        return true;
    }

    bool d3d12_renderer::create_sphere_buffers(const sphere_mesh& mesh)
    {
        const UINT64 vertex_size = static_cast<UINT64>(mesh.vertices.size() * sizeof(vertex));
        const UINT64 index_size = static_cast<UINT64>(mesh.indices.size() * sizeof(std::uint32_t));
        const D3D12_HEAP_PROPERTIES heap_properties = make_heap_properties(D3D12_HEAP_TYPE_UPLOAD);
        const D3D12_RESOURCE_DESC vertex_description = make_buffer_desc(vertex_size);
        const D3D12_RESOURCE_DESC index_description = make_buffer_desc(index_size);
        if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE, &vertex_description,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&vertex_buffer_)))) return false;
        if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE, &index_description,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&index_buffer_)))) return false;

        void* mapped = nullptr;
        D3D12_RANGE read_range{0, 0};
        if (FAILED(vertex_buffer_->Map(0, &read_range, &mapped))) return false;
        std::memcpy(mapped, mesh.vertices.data(), static_cast<std::size_t>(vertex_size));
        vertex_buffer_->Unmap(0, nullptr);
        if (FAILED(index_buffer_->Map(0, &read_range, &mapped))) return false;
        std::memcpy(mapped, mesh.indices.data(), static_cast<std::size_t>(index_size));
        index_buffer_->Unmap(0, nullptr);

        vertex_buffer_view_.BufferLocation = vertex_buffer_->GetGPUVirtualAddress();
        vertex_buffer_view_.SizeInBytes = static_cast<UINT>(vertex_size);
        vertex_buffer_view_.StrideInBytes = sizeof(vertex);
        index_buffer_view_.BufferLocation = index_buffer_->GetGPUVirtualAddress();
        index_buffer_view_.SizeInBytes = static_cast<UINT>(index_size);
        index_buffer_view_.Format = DXGI_FORMAT_R32_UINT;
        index_count_ = static_cast<std::uint32_t>(mesh.indices.size());
        return true;
    }

    bool d3d12_renderer::create_sphere_pipeline()
    {
        D3D12_ROOT_PARAMETER root_parameter{};
        root_parameter.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        root_parameter.Constants.ShaderRegister = 0;
        root_parameter.Constants.RegisterSpace = 0;
        root_parameter.Constants.Num32BitValues = 36;
        root_parameter.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

        D3D12_ROOT_SIGNATURE_DESC root_description{};
        root_description.NumParameters = 1;
        root_description.pParameters = &root_parameter;
        root_description.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        Microsoft::WRL::ComPtr<ID3DBlob> serialized_root;
        Microsoft::WRL::ComPtr<ID3DBlob> error_blob;
        if (FAILED(D3D12SerializeRootSignature(&root_description, D3D_ROOT_SIGNATURE_VERSION_1,
            &serialized_root, &error_blob))) return false;
        if (FAILED(device_->CreateRootSignature(0, serialized_root->GetBufferPointer(),
            serialized_root->GetBufferSize(), IID_PPV_ARGS(&graphics_root_signature_)))) return false;

        wchar_t executable_path[MAX_PATH]{};
        GetModuleFileNameW(nullptr, executable_path, MAX_PATH);
        const std::filesystem::path shader_directory = std::filesystem::path(executable_path).parent_path() / L"shaders";
        const std::vector<char> vertex_shader = read_binary_file(shader_directory / L"render_sphere_vs.dxil");
        const std::vector<char> pixel_shader = read_binary_file(shader_directory / L"render_sphere_ps.dxil");
        if (vertex_shader.empty() || pixel_shader.empty()) return false;

        const D3D12_INPUT_ELEMENT_DESC input_elements[] =
        {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC pipeline_description{};
        pipeline_description.pRootSignature = graphics_root_signature_.Get();
        pipeline_description.VS = {vertex_shader.data(), vertex_shader.size()};
        pipeline_description.PS = {pixel_shader.data(), pixel_shader.size()};
        pipeline_description.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        pipeline_description.SampleMask = UINT_MAX;
        pipeline_description.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        pipeline_description.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
        pipeline_description.RasterizerState.DepthClipEnable = TRUE;
        pipeline_description.DepthStencilState.DepthEnable = TRUE;
        pipeline_description.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        pipeline_description.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
        pipeline_description.InputLayout = {input_elements, static_cast<UINT>(std::size(input_elements))};
        pipeline_description.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        pipeline_description.NumRenderTargets = 1;
        pipeline_description.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        pipeline_description.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        pipeline_description.SampleDesc.Count = 1;
        return SUCCEEDED(device_->CreateGraphicsPipelineState(&pipeline_description, IID_PPV_ARGS(&graphics_pipeline_)));
    }

    void d3d12_renderer::render()
    {
        ID3D12CommandAllocator* allocator = command_allocators_[frame_index_].Get();
        if (FAILED(allocator->Reset())) throw std::runtime_error("Command allocator reset failed.");
        if (FAILED(command_list_->Reset(allocator, graphics_pipeline_.Get()))) throw std::runtime_error("Command list reset failed.");

        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource = render_targets_[frame_index_].Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        command_list_->ResourceBarrier(1, &barrier);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += static_cast<SIZE_T>(frame_index_) * rtv_descriptor_size_;
        const D3D12_CPU_DESCRIPTOR_HANDLE dsv = dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        command_list_->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
        constexpr float clear_color[4] = {0.025f, 0.035f, 0.055f, 1.0f};
        command_list_->ClearRenderTargetView(rtv, clear_color, 0, nullptr);
        command_list_->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

        D3D12_VIEWPORT viewport{};
        viewport.Width = static_cast<float>(width_);
        viewport.Height = static_cast<float>(height_);
        viewport.MaxDepth = 1.0f;
        const D3D12_RECT scissor{0, 0, static_cast<LONG>(width_), static_cast<LONG>(height_)};
        command_list_->RSSetViewports(1, &viewport);
        command_list_->RSSetScissorRects(1, &scissor);
        command_list_->SetGraphicsRootSignature(graphics_root_signature_.Get());
        command_list_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        command_list_->IASetVertexBuffers(0, 1, &vertex_buffer_view_);
        command_list_->IASetIndexBuffer(&index_buffer_view_);

        using namespace DirectX;
        const XMMATRIX view = XMMatrixLookAtLH(XMVectorSet(0.0f, 0.0f, -5.0f, 1.0f),
            XMVectorZero(), XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
        const XMMATRIX projection = XMMatrixPerspectiveFovLH(XM_PIDIV4,
            static_cast<float>(width_) / static_cast<float>(height_), 0.1f, 100.0f);
        draw_constants constants{};
        XMStoreFloat4x4(&constants.view_projection, view * projection);

        XMStoreFloat4x4(&constants.world, XMMatrixTranslation(-0.82f, 0.0f, 0.0f));
        constants.color = XMFLOAT4(0.15f, 0.55f, 0.95f, 1.0f);
        command_list_->SetGraphicsRoot32BitConstants(0, 36, &constants, 0);
        command_list_->DrawIndexedInstanced(index_count_, 1, 0, 0, 0);

        XMStoreFloat4x4(&constants.world, XMMatrixTranslation(0.82f, 0.0f, 0.0f));
        constants.color = XMFLOAT4(0.95f, 0.35f, 0.12f, 1.0f);
        command_list_->SetGraphicsRoot32BitConstants(0, 36, &constants, 0);
        command_list_->DrawIndexedInstanced(index_count_, 1, 0, 0, 0);

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
