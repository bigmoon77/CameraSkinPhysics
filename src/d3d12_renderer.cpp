#include "d3d12_renderer.hpp"
#include "benchmark_log.hpp"
#include "shape_matching.hpp"

#include <DirectXMath.h>
#include <d3d12sdklayers.h>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace csp
{
    namespace
    {
        constexpr std::uint32_t compute_srv_count = 8;
        constexpr std::uint32_t compute_uav_count = 2;
        constexpr std::uint32_t compute_table_size = compute_srv_count + compute_uav_count;
        constexpr std::uint32_t compute_descriptor_capacity = 4096;

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

        D3D12_RESOURCE_DESC make_buffer_desc(UINT64 size, D3D12_RESOURCE_FLAGS flags = D3D12_RESOURCE_FLAG_NONE)
        {
            D3D12_RESOURCE_DESC description{};
            description.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            description.Width = std::max<UINT64>(size, 4);
            description.Height = 1;
            description.DepthOrArraySize = 1;
            description.MipLevels = 1;
            description.Format = DXGI_FORMAT_UNKNOWN;
            description.SampleDesc.Count = 1;
            description.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
            description.Flags = flags;
            return description;
        }

        Microsoft::WRL::ComPtr<ID3D12Resource> create_buffer(
            ID3D12Device* device, UINT64 size, D3D12_HEAP_TYPE heap_type,
            D3D12_RESOURCE_FLAGS flags, D3D12_RESOURCE_STATES initial_state)
        {
            const D3D12_HEAP_PROPERTIES heap_properties = make_heap_properties(heap_type);
            const D3D12_RESOURCE_DESC description = make_buffer_desc(size, flags);
            Microsoft::WRL::ComPtr<ID3D12Resource> resource;
            if (FAILED(device->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE, &description,
                initial_state, nullptr, IID_PPV_ARGS(&resource)))) return {};
            return resource;
        }

        Microsoft::WRL::ComPtr<ID3D12Resource> create_upload_buffer(
            ID3D12Device* device, const void* data, UINT64 size)
        {
            Microsoft::WRL::ComPtr<ID3D12Resource> resource = create_buffer(device, size,
                D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_GENERIC_READ);
            if (!resource) return {};

            void* mapped = nullptr;
            const D3D12_RANGE read_range{0, 0};
            if (FAILED(resource->Map(0, &read_range, &mapped))) return {};
            std::memcpy(mapped, data, static_cast<std::size_t>(size));
            resource->Unmap(0, nullptr);
            return resource;
        }

        struct draw_constants
        {
            DirectX::XMFLOAT4X4 view_projection;
            DirectX::XMFLOAT4X4 world;
            DirectX::XMFLOAT4 color;
        };
        static_assert(sizeof(draw_constants) == 144);

        struct capture_constants
        {
            DirectX::XMFLOAT4X4 world;
            DirectX::XMFLOAT4X4 view_projection;
        };
        static_assert(sizeof(capture_constants) == 128);

        struct clear_constants
        {
            std::uint32_t element_count;
            float padding[3];
        };
        static_assert(sizeof(clear_constants) == 16);

        struct integrate_constants
        {
            std::uint32_t particle_count;
            float delta_time;
            float damping;
            float padding0;
            float gravity[3];
            float padding1;
        };
        static_assert(sizeof(integrate_constants) == 32);

        struct tetrahedron_constants
        {
            std::uint32_t tetrahedron_count;
            float delta_time;
            float relaxation;
            float padding;
        };
        static_assert(sizeof(tetrahedron_constants) == 16);

        struct gather_constants
        {
            std::uint32_t particle_count;
            float max_correction;
            float delta_time;
            float padding;
        };
        static_assert(sizeof(gather_constants) == 16);

        struct camera_area_constants
        {
            std::uint32_t triangle_count;
            std::uint32_t edge_count;
            float delta_time;
            float relaxation;
            float compliance;
        };
        static_assert(sizeof(camera_area_constants) == 20);

        struct camera_gather_constants
        {
            std::uint32_t particle_count;
            float max_correction;
            float delta_time;
            std::uint32_t edge_count;
            std::uint32_t triangle_count;
            std::uint32_t padding[3];
        };
        static_assert(sizeof(camera_gather_constants) == 32);

        struct camera_bending_constants
        {
            std::uint32_t hinge_count;
            std::uint32_t edge_count;
            std::uint32_t triangle_count;
            float delta_time;
            float relaxation;
            float compliance;
            float padding[2];
        };
        static_assert(sizeof(camera_bending_constants) == 32);

        struct skin_constants
        {
            std::uint32_t vertex_count;
            float local_center[3];
        };
        static_assert(sizeof(skin_constants) == 16);

        struct shape_build_constants
        {
            std::uint32_t cluster_count;
            std::uint32_t padding[3];
        };
        static_assert(sizeof(shape_build_constants) == 16);

        struct shape_apply_constants
        {
            std::uint32_t particle_count;
            float strength;
            float bone_follow_strength;
            float max_correction;
            float delta_time;
            float bone_center_x;
            float padding[2];
        };
        static_assert(sizeof(shape_apply_constants) == 32);
        static_assert(sizeof(DirectX::XMFLOAT4) * 3u == 48u);

        struct camera_collision_constants
        {
            DirectX::XMFLOAT4X4 view_projection;
            DirectX::XMFLOAT4X4 inverse_view_projection;
            std::uint32_t particle_count;
            std::uint32_t viewport_width;
            std::uint32_t viewport_height;
            float particle_radius;
            float pushout;
            float velocity_friction;
            float restitution;
            float contact_band;
            std::uint32_t smooth_surface_contacts;
        };
        static_assert(sizeof(camera_collision_constants) == 164);

        void transition_barrier(ID3D12GraphicsCommandList* command_list,
            ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after)
        {
            if (resource == nullptr || before == after) return;
            D3D12_RESOURCE_BARRIER barrier{};
            barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barrier.Transition.pResource = resource;
            barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            barrier.Transition.StateBefore = before;
            barrier.Transition.StateAfter = after;
            command_list->ResourceBarrier(1, &barrier);
        }
    }

    struct d3d12_renderer::compute_state
    {
        struct gpu_body
        {
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> particles;
            Microsoft::WRL::ComPtr<ID3D12Resource> tetrahedron_corrections;
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> tetrahedron_lambdas;
            Microsoft::WRL::ComPtr<ID3D12Resource> vertices;
            D3D12_VERTEX_BUFFER_VIEW vertex_view{};
            std::uint32_t particle_read = 0;
            bool correction_is_srv = false;
            bool lambda_zero_is_srv = false;
            bool vertex_is_input_buffer = false;
            float center_x = 0.0f;
        };

        struct camera_gpu_body
        {
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> particles;
            Microsoft::WRL::ComPtr<ID3D12Resource> edge_corrections;
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> contact_corrections;
            std::array<bool, 2> contact_correction_is_srv{};
            std::uint32_t contact_correction_read = 0;
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> edge_lambdas;
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> area_lambdas;
            std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> bending_lambdas;
            Microsoft::WRL::ComPtr<ID3D12Resource> shape_cluster_transforms;
            Microsoft::WRL::ComPtr<ID3D12Resource> vertices;
            D3D12_VERTEX_BUFFER_VIEW vertex_view{};
            std::uint32_t particle_read = 0;
            bool correction_is_srv = false;
            bool lambda_zero_is_srv = false;
            bool area_lambda_zero_is_srv = false;
            bool bending_lambda_zero_is_srv = false;
            bool shape_transforms_is_srv = false;
            bool vertex_is_input_buffer = false;
            float center_x = 0.0f;
        };

        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> descriptor_heap;
        std::uint32_t descriptor_size = 0;
        std::uint32_t descriptor_cursor = 0;
        std::uint32_t particle_count = 0;
        std::uint32_t tetrahedron_count = 0;
        std::uint32_t vertex_count = 0;
        std::uint32_t edge_count = 0;
        std::uint32_t triangle_count = 0;
        std::uint32_t bending_count = 0;
        std::uint32_t shape_cluster_count = 0;
        std::uint32_t shape_membership_count = 0;
        Microsoft::WRL::ComPtr<ID3D12Resource> inverse_masses;
        Microsoft::WRL::ComPtr<ID3D12Resource> rest_positions;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_rest_positions;
        DirectX::XMFLOAT4* mapped_surface_rest_positions = nullptr;
        surface_edge* mapped_surface_edges = nullptr;
        triangle_area_constraint* mapped_surface_triangle_areas = nullptr;
        bending_constraint* mapped_surface_bending = nullptr;
        skeletal_animation animation;
        std::vector<std::array<float, 3>> skeletal_rest_positions;
        std::vector<skin_vertex_weights> skeletal_vertex_weights;
        std::vector<skin_matrix> sampled_skin_matrices;
        float animation_time = 0.0f;
        Microsoft::WRL::ComPtr<ID3D12Resource> shape_cluster_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> shape_cluster_vertices;
        Microsoft::WRL::ComPtr<ID3D12Resource> vertex_cluster_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> vertex_clusters;
        Microsoft::WRL::ComPtr<ID3D12Resource> tetrahedra;
        Microsoft::WRL::ComPtr<ID3D12Resource> incidence_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> incident_tetrahedra;
        Microsoft::WRL::ComPtr<ID3D12Resource> tetrahedron_indices;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_mappings;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_edges;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_edge_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> incident_surface_corrections;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_triangle_areas;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_triangle_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> incident_surface_triangles;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_bending;
        Microsoft::WRL::ComPtr<ID3D12Resource> surface_bending_ranges;
        Microsoft::WRL::ComPtr<ID3D12Resource> incident_surface_bending;
        std::array<gpu_body, 2> bodies;
        std::array<camera_gpu_body, 2> camera_bodies;
        Microsoft::WRL::ComPtr<ID3D12RootSignature> root_signature;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> integrate_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> clear_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> tetrahedron_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> gather_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> skin_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_collision_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_contact_apply_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_edge_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_area_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_bending_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_gather_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_skin_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_shape_build_pipeline;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> camera_surface_shape_apply_pipeline;
        std::array<std::unique_ptr<benchmark_log>, 2> logs;
        std::array<std::uint64_t, 2> benchmark_frame_numbers{};
        std::uint64_t tetrahedral_buffer_bytes = 0;
        std::uint64_t camera_surface_buffer_bytes = 0;
        std::uint64_t resident_simulation_buffer_bytes = 0;
    };

    d3d12_renderer::d3d12_renderer() = default;

    d3d12_renderer::~d3d12_renderer()
    {
        wait_gpu();
        if (compute_)
        {
            if (compute_->surface_rest_positions && compute_->mapped_surface_rest_positions)
                compute_->surface_rest_positions->Unmap(0, nullptr);
            if (compute_->surface_edges && compute_->mapped_surface_edges)
                compute_->surface_edges->Unmap(0, nullptr);
            if (compute_->surface_triangle_areas && compute_->mapped_surface_triangle_areas)
                compute_->surface_triangle_areas->Unmap(0, nullptr);
            if (compute_->surface_bending && compute_->mapped_surface_bending)
                compute_->surface_bending->Unmap(0, nullptr);
        }
        if (fence_event_ != nullptr) CloseHandle(fence_event_);
    }

    bool d3d12_renderer::initialize(HWND window, std::uint32_t width, std::uint32_t height,
        const sphere_mesh& surface_mesh, const tetrahedral_mesh& volume_mesh,
        const skeletal_animation* animation)
    {
        width_ = width;
        height_ = height;
        if (width_ == 0 || height_ == 0) return false;

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
        swap_description.Width = width_;
        swap_description.Height = height_;
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
        dsv_descriptor_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

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
        create_capture_targets();
        if (!create_depth_buffer()
            || !create_sphere_buffers(surface_mesh)
            || !create_sphere_pipeline()
            || !create_capture_pipeline()
            || !create_compute_pipelines()
            || !create_compute_resources(surface_mesh, volume_mesh, animation)) return false;

        D3D12_QUERY_HEAP_DESC query_description{};
        query_description.Count = 5u + max_solver_iterations * 2u * 8u;
        query_description.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
        if (FAILED(device_->CreateQueryHeap(&query_description, IID_PPV_ARGS(&timestamp_query_heap_)))) return false;
        timestamp_readback_ = create_buffer(device_.Get(), sizeof(std::uint64_t) * (5u + max_solver_iterations * 2u * 8u),
            D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST);
        if (!timestamp_readback_) return false;
        if (FAILED(command_queue_->GetTimestampFrequency(&timestamp_frequency_))) timestamp_frequency_ = 0;

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

    void d3d12_renderer::create_capture_targets()
    {
        D3D12_DESCRIPTOR_HEAP_DESC rtv_description{};
        rtv_description.NumDescriptors = capture_count;
        rtv_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        if (FAILED(device_->CreateDescriptorHeap(&rtv_description, IID_PPV_ARGS(&capture_rtv_heap_))))
            throw std::runtime_error("Failed to create capture RTV heap.");
        capture_rtv_descriptor_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        D3D12_DESCRIPTOR_HEAP_DESC dsv_description{};
        dsv_description.NumDescriptors = capture_count;
        dsv_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        if (FAILED(device_->CreateDescriptorHeap(&dsv_description, IID_PPV_ARGS(&capture_dsv_heap_))))
            throw std::runtime_error("Failed to create capture DSV heap.");
        capture_dsv_descriptor_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

        const D3D12_HEAP_PROPERTIES heap_properties = make_heap_properties(D3D12_HEAP_TYPE_DEFAULT);
        D3D12_RESOURCE_DESC normal_description{};
        normal_description.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        normal_description.Width = capture_size;
        normal_description.Height = capture_size;
        normal_description.DepthOrArraySize = 1;
        normal_description.MipLevels = 1;
        normal_description.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        normal_description.SampleDesc.Count = 1;
        normal_description.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        normal_description.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

        D3D12_CLEAR_VALUE normal_clear{};
        normal_clear.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        normal_clear.Color[0] = 0.5f;
        normal_clear.Color[1] = 0.5f;
        normal_clear.Color[2] = 1.0f;
        normal_clear.Color[3] = 1.0f;

        D3D12_RESOURCE_DESC depth_description = normal_description;
        depth_description.Format = DXGI_FORMAT_R32_TYPELESS;
        depth_description.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        D3D12_CLEAR_VALUE depth_clear{};
        depth_clear.Format = DXGI_FORMAT_D32_FLOAT;
        depth_clear.DepthStencil.Depth = 1.0f;
        depth_clear.DepthStencil.Stencil = 0;

        D3D12_CPU_DESCRIPTOR_HANDLE rtv_handle = capture_rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        D3D12_CPU_DESCRIPTOR_HANDLE dsv_handle = capture_dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        for (std::uint32_t index = 0; index < capture_count; ++index)
        {
            if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE,
                &normal_description, D3D12_RESOURCE_STATE_RENDER_TARGET, &normal_clear,
                IID_PPV_ARGS(&capture_normal_[index]))))
                throw std::runtime_error("Failed to create capture normal texture.");
            if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE,
                &depth_description, D3D12_RESOURCE_STATE_DEPTH_WRITE, &depth_clear,
                IID_PPV_ARGS(&capture_depth_[index]))))
                throw std::runtime_error("Failed to create capture depth texture.");

            device_->CreateRenderTargetView(capture_normal_[index].Get(), nullptr, rtv_handle);
            D3D12_DEPTH_STENCIL_VIEW_DESC dsv_view{};
            dsv_view.Format = DXGI_FORMAT_D32_FLOAT;
            dsv_view.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
            device_->CreateDepthStencilView(capture_depth_[index].Get(), &dsv_view, dsv_handle);

            rtv_handle.ptr += capture_rtv_descriptor_size_;
            dsv_handle.ptr += capture_dsv_descriptor_size_;
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
        const D3D12_HEAP_PROPERTIES heap_properties = make_heap_properties(D3D12_HEAP_TYPE_DEFAULT);
        if (FAILED(device_->CreateCommittedResource(&heap_properties, D3D12_HEAP_FLAG_NONE, &description,
            D3D12_RESOURCE_STATE_DEPTH_WRITE, &clear_value, IID_PPV_ARGS(&depth_buffer_)))) return false;
        device_->CreateDepthStencilView(depth_buffer_.Get(), nullptr, dsv_heap_->GetCPUDescriptorHandleForHeapStart());
        return true;
    }

    bool d3d12_renderer::create_sphere_buffers(const sphere_mesh& mesh)
    {
        const UINT64 index_size = static_cast<UINT64>(mesh.indices.size() * sizeof(std::uint32_t));
        index_buffer_ = create_upload_buffer(device_.Get(), mesh.indices.data(), index_size);
        if (!index_buffer_) return false;

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

    bool d3d12_renderer::create_capture_pipeline()
    {
        D3D12_ROOT_PARAMETER root_parameter{};
        root_parameter.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        root_parameter.Constants.ShaderRegister = 0;
        root_parameter.Constants.RegisterSpace = 0;
        root_parameter.Constants.Num32BitValues = 32;
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
            serialized_root->GetBufferSize(), IID_PPV_ARGS(&capture_root_signature_)))) return false;

        wchar_t executable_path[MAX_PATH]{};
        GetModuleFileNameW(nullptr, executable_path, MAX_PATH);
        const std::filesystem::path shader_directory = std::filesystem::path(executable_path).parent_path() / L"shaders";
        const std::vector<char> vertex_shader = read_binary_file(shader_directory / L"capture_surface_vs.dxil");
        const std::vector<char> pixel_shader = read_binary_file(shader_directory / L"capture_surface_ps.dxil");
        if (vertex_shader.empty() || pixel_shader.empty()) return false;

        const D3D12_INPUT_ELEMENT_DESC input_elements[] =
        {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC pipeline_description{};
        pipeline_description.pRootSignature = capture_root_signature_.Get();
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
        pipeline_description.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
        pipeline_description.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        pipeline_description.SampleDesc.Count = 1;
        return SUCCEEDED(device_->CreateGraphicsPipelineState(&pipeline_description, IID_PPV_ARGS(&capture_pipeline_)));
    }

    bool d3d12_renderer::create_compute_pipelines()
    {
        compute_ = std::make_unique<compute_state>();

        D3D12_DESCRIPTOR_HEAP_DESC heap_description{};
        heap_description.NumDescriptors = compute_descriptor_capacity;
        heap_description.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        heap_description.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        if (FAILED(device_->CreateDescriptorHeap(&heap_description, IID_PPV_ARGS(&compute_->descriptor_heap)))) return false;
        compute_->descriptor_size = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

        D3D12_DESCRIPTOR_RANGE ranges[2]{};
        ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
        ranges[0].NumDescriptors = compute_srv_count;
        ranges[0].BaseShaderRegister = 0;
        ranges[0].RegisterSpace = 0;
        ranges[0].OffsetInDescriptorsFromTableStart = 0;
        ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
        ranges[1].NumDescriptors = compute_uav_count;
        ranges[1].BaseShaderRegister = 0;
        ranges[1].RegisterSpace = 0;
        ranges[1].OffsetInDescriptorsFromTableStart = compute_srv_count;

        D3D12_ROOT_PARAMETER parameters[2]{};
        parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        parameters[0].Constants.ShaderRegister = 0;
        parameters[0].Constants.RegisterSpace = 0;
        parameters[0].Constants.Num32BitValues = 41;
        parameters[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        parameters[1].DescriptorTable.NumDescriptorRanges = 2;
        parameters[1].DescriptorTable.pDescriptorRanges = ranges;
        parameters[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

        D3D12_ROOT_SIGNATURE_DESC root_description{};
        root_description.NumParameters = 2;
        root_description.pParameters = parameters;
        Microsoft::WRL::ComPtr<ID3DBlob> serialized_root;
        Microsoft::WRL::ComPtr<ID3DBlob> error_blob;
        if (FAILED(D3D12SerializeRootSignature(&root_description, D3D_ROOT_SIGNATURE_VERSION_1,
            &serialized_root, &error_blob))) return false;
        if (FAILED(device_->CreateRootSignature(0, serialized_root->GetBufferPointer(),
            serialized_root->GetBufferSize(), IID_PPV_ARGS(&compute_->root_signature)))) return false;

        wchar_t executable_path[MAX_PATH]{};
        GetModuleFileNameW(nullptr, executable_path, MAX_PATH);
        const std::filesystem::path shader_directory = std::filesystem::path(executable_path).parent_path() / L"shaders";
        const auto create_pipeline = [this, &shader_directory](const wchar_t* name,
            Microsoft::WRL::ComPtr<ID3D12PipelineState>& pipeline) -> bool
        {
            const std::vector<char> shader = read_binary_file(shader_directory / name);
            if (shader.empty()) return false;
            D3D12_COMPUTE_PIPELINE_STATE_DESC description{};
            description.pRootSignature = compute_->root_signature.Get();
            description.CS = {shader.data(), shader.size()};
            return SUCCEEDED(device_->CreateComputePipelineState(&description, IID_PPV_ARGS(&pipeline)));
        };

        return create_pipeline(L"integrate_particles.dxil", compute_->integrate_pipeline)
            && create_pipeline(L"clear_lambdas.dxil", compute_->clear_pipeline)
            && create_pipeline(L"xpbd_tetrahedra.dxil", compute_->tetrahedron_pipeline)
            && create_pipeline(L"xpbd_gather.dxil", compute_->gather_pipeline)
            && create_pipeline(L"skin_vertices.dxil", compute_->skin_pipeline)
            && create_pipeline(L"camera_collision.dxil", compute_->camera_collision_pipeline)
            && create_pipeline(L"camera_contact_apply.dxil", compute_->camera_contact_apply_pipeline)
            && create_pipeline(L"camera_surface_edges.dxil", compute_->camera_surface_edge_pipeline)
            && create_pipeline(L"camera_surface_area.dxil", compute_->camera_surface_area_pipeline)
            && create_pipeline(L"camera_surface_bending.dxil", compute_->camera_surface_bending_pipeline)
            && create_pipeline(L"camera_surface_gather.dxil", compute_->camera_surface_gather_pipeline)
            && create_pipeline(L"camera_surface_skin.dxil", compute_->camera_surface_skin_pipeline)
            && create_pipeline(L"camera_surface_shape_build.dxil", compute_->camera_surface_shape_build_pipeline)
            && create_pipeline(L"camera_surface_shape_apply.dxil", compute_->camera_surface_shape_apply_pipeline);
    }

    bool d3d12_renderer::create_compute_resources(const sphere_mesh& surface_mesh,
        const tetrahedral_mesh& volume_mesh, const skeletal_animation* animation)
    {
        compute_->particle_count = static_cast<std::uint32_t>(volume_mesh.particles.size());
        compute_->tetrahedron_count = static_cast<std::uint32_t>(volume_mesh.tetrahedra.size());
        compute_->vertex_count = static_cast<std::uint32_t>(surface_mesh.vertices.size());
        compute_->edge_count = static_cast<std::uint32_t>(surface_mesh.edges.size());
        compute_->triangle_count = static_cast<std::uint32_t>(surface_mesh.triangle_areas.size());
        compute_->bending_count = static_cast<std::uint32_t>(surface_mesh.bending.size());
        const shape_matching_clusters shape_clusters = build_shape_matching_clusters(surface_mesh, 2048u, 2u);
        compute_->shape_cluster_count = static_cast<std::uint32_t>(shape_clusters.cluster_vertex_ranges.size());
        compute_->shape_membership_count = static_cast<std::uint32_t>(shape_clusters.vertex_clusters.size());
        if (compute_->particle_count == 0 || compute_->tetrahedron_count == 0
            || compute_->shape_cluster_count == 0 || compute_->shape_membership_count < compute_->vertex_count
            || compute_->vertex_count == 0 || compute_->edge_count == 0 || compute_->triangle_count == 0
            || surface_mesh.edge_incidence_ranges.size() != surface_mesh.vertices.size()
            || surface_mesh.incident_corrections.size() != surface_mesh.edges.size() * 2
            || surface_mesh.triangle_incidence_ranges.size() != surface_mesh.vertices.size()
            || surface_mesh.incident_triangle_corners.size() != surface_mesh.triangle_areas.size() * 3
            || surface_mesh.bending_incidence_ranges.size() != surface_mesh.vertices.size()
            || surface_mesh.incident_bending_corners.size() != surface_mesh.bending.size() * 4)
            return false;

        const auto upload_vector = [this](const auto& values) -> Microsoft::WRL::ComPtr<ID3D12Resource>
        {
            if (values.empty()) return {};
            return create_upload_buffer(device_.Get(), values.data(), values.size() * sizeof(values[0]));
        };

        compute_->inverse_masses = upload_vector(volume_mesh.inverse_masses);
        std::vector<DirectX::XMFLOAT4> rest_positions;
        rest_positions.reserve(volume_mesh.particles.size());
        for (const simulation_particle& particle : volume_mesh.particles)
            rest_positions.emplace_back(particle.position[0], particle.position[1], particle.position[2], 1.0f);
        compute_->rest_positions = create_upload_buffer(device_.Get(), rest_positions.data(),
            rest_positions.size() * sizeof(DirectX::XMFLOAT4));
        std::vector<DirectX::XMFLOAT4> surface_rest_positions;
        surface_rest_positions.reserve(surface_mesh.vertices.size());
        for (const vertex& surface_vertex : surface_mesh.vertices)
            surface_rest_positions.emplace_back(surface_vertex.position[0], surface_vertex.position[1],
                surface_vertex.position[2], 1.0f);
        compute_->surface_rest_positions = create_upload_buffer(device_.Get(), surface_rest_positions.data(),
            surface_rest_positions.size() * sizeof(DirectX::XMFLOAT4));
        compute_->skeletal_rest_positions.reserve(surface_mesh.vertices.size());
        for (const vertex& source_vertex : surface_mesh.vertices)
            compute_->skeletal_rest_positions.push_back({source_vertex.position[0],
                source_vertex.position[1], source_vertex.position[2]});
        if (animation != nullptr && !animation->frames.empty())
        {
            if (animation->vertex_weights.size() != surface_mesh.vertices.size()
                || animation->frames.front().size() != animation->bones.size()) return false;
            compute_->animation = *animation;
            compute_->skeletal_vertex_weights = animation->vertex_weights;
        }
        compute_->shape_cluster_ranges = upload_vector(shape_clusters.cluster_vertex_ranges);
        compute_->shape_cluster_vertices = upload_vector(shape_clusters.cluster_vertices);
        compute_->vertex_cluster_ranges = upload_vector(shape_clusters.vertex_cluster_ranges);
        compute_->vertex_clusters = upload_vector(shape_clusters.vertex_clusters);
        compute_->tetrahedra = upload_vector(volume_mesh.tetrahedra);
        compute_->incidence_ranges = upload_vector(volume_mesh.incidence_ranges);
        compute_->incident_tetrahedra = upload_vector(volume_mesh.incident_tetrahedra);
        compute_->tetrahedron_indices = upload_vector(volume_mesh.tetrahedron_indices);
        compute_->surface_mappings = upload_vector(volume_mesh.surface_mappings);
        compute_->surface_edges = upload_vector(surface_mesh.edges);
        compute_->surface_edge_ranges = upload_vector(surface_mesh.edge_incidence_ranges);
        compute_->incident_surface_corrections = upload_vector(surface_mesh.incident_corrections);
        compute_->surface_triangle_areas = upload_vector(surface_mesh.triangle_areas);
        compute_->surface_triangle_ranges = upload_vector(surface_mesh.triangle_incidence_ranges);
        compute_->incident_surface_triangles = upload_vector(surface_mesh.incident_triangle_corners);
        std::vector<bending_constraint> bending_upload = surface_mesh.bending;
        if (bending_upload.empty()) bending_upload.emplace_back();
        std::vector<std::uint32_t> incident_bending_upload = surface_mesh.incident_bending_corners;
        if (incident_bending_upload.empty()) incident_bending_upload.push_back(0u);
        compute_->surface_bending = upload_vector(bending_upload);
        compute_->surface_bending_ranges = upload_vector(surface_mesh.bending_incidence_ranges);
        compute_->incident_surface_bending = upload_vector(incident_bending_upload);


        if (!compute_->inverse_masses || !compute_->rest_positions || !compute_->tetrahedra
            || !compute_->incidence_ranges || !compute_->incident_tetrahedra
            || !compute_->tetrahedron_indices || !compute_->surface_mappings
            || !compute_->surface_rest_positions || !compute_->shape_cluster_ranges
            || !compute_->shape_cluster_vertices || !compute_->vertex_cluster_ranges || !compute_->vertex_clusters
            || !compute_->surface_edges || !compute_->surface_edge_ranges || !compute_->incident_surface_corrections
            || !compute_->surface_triangle_areas || !compute_->surface_triangle_ranges
            || !compute_->incident_surface_triangles || !compute_->surface_bending
            || !compute_->surface_bending_ranges || !compute_->incident_surface_bending)
            return false;

        const D3D12_RANGE no_read_range{0, 0};
        void* mapped_data = nullptr;
        if (FAILED(compute_->surface_rest_positions->Map(0, &no_read_range, &mapped_data))) return false;
        compute_->mapped_surface_rest_positions = static_cast<DirectX::XMFLOAT4*>(mapped_data);
        mapped_data = nullptr;
        if (FAILED(compute_->surface_edges->Map(0, &no_read_range, &mapped_data))) return false;
        compute_->mapped_surface_edges = static_cast<surface_edge*>(mapped_data);
        mapped_data = nullptr;
        if (FAILED(compute_->surface_triangle_areas->Map(0, &no_read_range, &mapped_data))) return false;
        compute_->mapped_surface_triangle_areas = static_cast<triangle_area_constraint*>(mapped_data);
        mapped_data = nullptr;
        if (FAILED(compute_->surface_bending->Map(0, &no_read_range, &mapped_data))) return false;
        compute_->mapped_surface_bending = static_cast<bending_constraint*>(mapped_data);

        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> volume_uploads;
        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> surface_uploads;

        for (std::uint32_t body_index = 0; body_index < 2; ++body_index)
        {
            const float center_x = body_index == 0 ? -1.08f : 1.08f;
            const float initial_velocity_x = body_index == 0 ? 0.65f : -0.65f;

            compute_state::gpu_body& volume_body = compute_->bodies[body_index];
            volume_body.center_x = center_x;
            std::vector<simulation_particle> volume_particles = volume_mesh.particles;
            for (simulation_particle& particle : volume_particles)
            {
                particle.position[0] += center_x;
                particle.velocity[0] = initial_velocity_x;
            }

            const UINT64 volume_particle_bytes =
                static_cast<UINT64>(volume_particles.size()) * sizeof(simulation_particle);
            volume_uploads[body_index] = create_upload_buffer(device_.Get(),
                volume_particles.data(), volume_particle_bytes);
            if (!volume_uploads[body_index]) return false;

            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                volume_body.particles[buffer_index] = create_buffer(device_.Get(), volume_particle_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_COPY_DEST);
                if (!volume_body.particles[buffer_index]) return false;
            }

            const UINT64 tetrahedron_correction_bytes =
                static_cast<UINT64>(compute_->tetrahedron_count) * 4 * sizeof(DirectX::XMFLOAT4);
            const UINT64 tetrahedron_lambda_bytes =
                static_cast<UINT64>(compute_->tetrahedron_count) * 7 * sizeof(float);
            const UINT64 vertex_bytes = static_cast<UINT64>(compute_->vertex_count) * sizeof(vertex);

            volume_body.tetrahedron_corrections = create_buffer(device_.Get(), tetrahedron_correction_bytes,
                D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                volume_body.tetrahedron_lambdas[buffer_index] = create_buffer(device_.Get(), tetrahedron_lambda_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            }
            volume_body.vertices = create_buffer(device_.Get(), vertex_bytes, D3D12_HEAP_TYPE_DEFAULT,
                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            if (!volume_body.tetrahedron_corrections || !volume_body.tetrahedron_lambdas[0]
                || !volume_body.tetrahedron_lambdas[1] || !volume_body.vertices) return false;

            volume_body.vertex_view.BufferLocation = volume_body.vertices->GetGPUVirtualAddress();
            volume_body.vertex_view.SizeInBytes = static_cast<UINT>(vertex_bytes);
            volume_body.vertex_view.StrideInBytes = sizeof(vertex);

            compute_state::camera_gpu_body& camera_body = compute_->camera_bodies[body_index];
            camera_body.center_x = center_x;
            std::vector<simulation_particle> surface_particles(compute_->vertex_count);
            for (std::uint32_t vertex_index = 0; vertex_index < compute_->vertex_count; ++vertex_index)
            {
                const vertex& source = surface_mesh.vertices[vertex_index];
                simulation_particle& particle = surface_particles[vertex_index];
                particle.position[0] = source.position[0] + center_x;
                particle.position[1] = source.position[1];
                particle.position[2] = source.position[2];
                particle.position[3] = 1.0f;
                particle.velocity[0] = initial_velocity_x;
                particle.velocity[1] = 0.0f;
                particle.velocity[2] = 0.0f;
                particle.velocity[3] = 0.0f;
            }

            const UINT64 surface_particle_bytes =
                static_cast<UINT64>(surface_particles.size()) * sizeof(simulation_particle);
            surface_uploads[body_index] = create_upload_buffer(device_.Get(),
                surface_particles.data(), surface_particle_bytes);
            if (!surface_uploads[body_index]) return false;

            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                camera_body.particles[buffer_index] = create_buffer(device_.Get(), surface_particle_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_COPY_DEST);
                if (!camera_body.particles[buffer_index]) return false;
            }

            const UINT64 edge_correction_bytes =
                static_cast<UINT64>(compute_->edge_count + compute_->triangle_count * 3u
                    + compute_->bending_count * 4u) * sizeof(DirectX::XMFLOAT3);
            const UINT64 edge_lambda_bytes = static_cast<UINT64>(compute_->edge_count) * sizeof(float);
            const UINT64 area_lambda_bytes = static_cast<UINT64>(compute_->triangle_count) * sizeof(float);
            const UINT64 bending_lambda_bytes = static_cast<UINT64>(compute_->bending_count) * sizeof(float);
            const UINT64 shape_transform_bytes =
                static_cast<UINT64>(compute_->shape_cluster_count) * sizeof(DirectX::XMFLOAT4) * 3u;
            camera_body.shape_cluster_transforms = create_buffer(device_.Get(), shape_transform_bytes,
                D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            camera_body.edge_corrections = create_buffer(device_.Get(), edge_correction_bytes,
                D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            const UINT64 contact_correction_bytes =
                static_cast<UINT64>(compute_->vertex_count) * sizeof(DirectX::XMFLOAT4);
            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                camera_body.contact_corrections[buffer_index] = create_buffer(device_.Get(),
                    contact_correction_bytes, D3D12_HEAP_TYPE_DEFAULT,
                    D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                if (!camera_body.contact_corrections[buffer_index]) return false;
            }
            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                camera_body.edge_lambdas[buffer_index] = create_buffer(device_.Get(), edge_lambda_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                camera_body.area_lambdas[buffer_index] = create_buffer(device_.Get(), area_lambda_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                camera_body.bending_lambdas[buffer_index] = create_buffer(device_.Get(), bending_lambda_bytes,
                    D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            }
            camera_body.vertices = create_buffer(device_.Get(), vertex_bytes, D3D12_HEAP_TYPE_DEFAULT,
                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            if (!camera_body.edge_corrections || !camera_body.edge_lambdas[0]
                || !camera_body.edge_lambdas[1] || !camera_body.area_lambdas[0]
                || !camera_body.area_lambdas[1] || !camera_body.bending_lambdas[0]
                || !camera_body.bending_lambdas[1] || !camera_body.shape_cluster_transforms
                || !camera_body.vertices) return false;

            camera_body.vertex_view.BufferLocation = camera_body.vertices->GetGPUVirtualAddress();
            camera_body.vertex_view.SizeInBytes = static_cast<UINT>(vertex_bytes);
            camera_body.vertex_view.StrideInBytes = sizeof(vertex);
        }

        if (FAILED(command_allocators_[0]->Reset())) return false;
        if (FAILED(command_list_->Reset(command_allocators_[0].Get(), nullptr))) return false;
        for (std::uint32_t body_index = 0; body_index < 2; ++body_index)
        {
            for (std::uint32_t buffer_index = 0; buffer_index < 2; ++buffer_index)
            {
                ID3D12Resource* volume_destination =
                    compute_->bodies[body_index].particles[buffer_index].Get();
                command_list_->CopyBufferRegion(volume_destination, 0, volume_uploads[body_index].Get(),
                    0, static_cast<UINT64>(compute_->particle_count) * sizeof(simulation_particle));
                transition_barrier(command_list_.Get(), volume_destination,
                    D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

                ID3D12Resource* camera_destination =
                    compute_->camera_bodies[body_index].particles[buffer_index].Get();
                command_list_->CopyBufferRegion(camera_destination, 0, surface_uploads[body_index].Get(),
                    0, static_cast<UINT64>(compute_->vertex_count) * sizeof(simulation_particle));
                transition_barrier(command_list_.Get(), camera_destination,
                    D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            }
        }
        if (FAILED(command_list_->Close())) return false;
        ID3D12CommandList* command_lists[] = {command_list_.Get()};
        command_queue_->ExecuteCommandLists(1, command_lists);
        const std::uint64_t signal_value = next_fence_value_++;
        if (FAILED(command_queue_->Signal(fence_.Get(), signal_value))) return false;
        if (fence_->GetCompletedValue() < signal_value)
        {
            if (FAILED(fence_->SetEventOnCompletion(signal_value, fence_event_))) return false;
            WaitForSingleObject(fence_event_, INFINITE);
        }

        const auto resource_bytes = [](ID3D12Resource* resource) -> std::uint64_t
        {
            return resource != nullptr ? resource->GetDesc().Width : 0ull;
        };

        for (const compute_state::gpu_body& body : compute_->bodies)
        {
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.particles[0].Get());
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.particles[1].Get());
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.tetrahedron_corrections.Get());
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.tetrahedron_lambdas[0].Get());
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.tetrahedron_lambdas[1].Get());
            compute_->tetrahedral_buffer_bytes += resource_bytes(body.vertices.Get());
        }
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->inverse_masses.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->rest_positions.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->tetrahedra.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->incidence_ranges.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->incident_tetrahedra.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->tetrahedron_indices.Get());
        compute_->tetrahedral_buffer_bytes += resource_bytes(compute_->surface_mappings.Get());

        for (const compute_state::camera_gpu_body& body : compute_->camera_bodies)
        {
            compute_->camera_surface_buffer_bytes += resource_bytes(body.particles[0].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.particles[1].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.edge_corrections.Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.contact_corrections[0].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.contact_corrections[1].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.edge_lambdas[0].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.edge_lambdas[1].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.area_lambdas[0].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.area_lambdas[1].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.bending_lambdas[0].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.bending_lambdas[1].Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.shape_cluster_transforms.Get());
            compute_->camera_surface_buffer_bytes += resource_bytes(body.vertices.Get());
        }
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_edges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_edge_ranges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->incident_surface_corrections.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_triangle_areas.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_triangle_ranges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->incident_surface_triangles.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_bending.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_bending_ranges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->incident_surface_bending.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->surface_rest_positions.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->shape_cluster_ranges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->shape_cluster_vertices.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->vertex_cluster_ranges.Get());
        compute_->camera_surface_buffer_bytes += resource_bytes(compute_->vertex_clusters.Get());
        compute_->resident_simulation_buffer_bytes =
            compute_->tetrahedral_buffer_bytes + compute_->camera_surface_buffer_bytes;

        try
        {
            compute_->logs[0] = std::make_unique<benchmark_log>("output/benchmark_xpbd.csv");
            compute_->logs[1] = std::make_unique<benchmark_log>("output/benchmark_camera_surface.csv");
        }
        catch (const std::exception&)
        {
            return false;
        }
        return true;
    }

    std::uint32_t d3d12_renderer::reserve_descriptor_table()
    {
        if (!compute_ || compute_->descriptor_cursor + compute_table_size > compute_descriptor_capacity)
            throw std::runtime_error("Compute descriptor heap capacity exceeded.");
        const std::uint32_t base = compute_->descriptor_cursor;
        compute_->descriptor_cursor += compute_table_size;
        return base;
    }

    void d3d12_renderer::clear_descriptor_table(std::uint32_t base)
    {
        D3D12_CPU_DESCRIPTOR_HANDLE handle = compute_->descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<SIZE_T>(base) * compute_->descriptor_size;
        for (std::uint32_t index = 0; index < compute_srv_count; ++index)
        {
            D3D12_SHADER_RESOURCE_VIEW_DESC description{};
            description.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            description.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
            description.Format = DXGI_FORMAT_R32_FLOAT;
            description.Buffer.NumElements = 1;
            device_->CreateShaderResourceView(nullptr, &description, handle);
            handle.ptr += compute_->descriptor_size;
        }
        for (std::uint32_t index = 0; index < compute_uav_count; ++index)
        {
            D3D12_UNORDERED_ACCESS_VIEW_DESC description{};
            description.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
            description.Format = DXGI_FORMAT_R32_FLOAT;
            description.Buffer.NumElements = 1;
            device_->CreateUnorderedAccessView(nullptr, nullptr, &description, handle);
            handle.ptr += compute_->descriptor_size;
        }
    }

    void d3d12_renderer::write_buffer_srv(std::uint32_t base, std::uint32_t slot,
        ID3D12Resource* resource, std::uint32_t element_count, std::uint32_t stride)
    {
        if (resource == nullptr || slot >= compute_srv_count) return;
        D3D12_SHADER_RESOURCE_VIEW_DESC description{};
        description.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        description.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.Buffer.FirstElement = 0;
        description.Buffer.NumElements = element_count;
        description.Buffer.StructureByteStride = stride;
        D3D12_CPU_DESCRIPTOR_HANDLE handle = compute_->descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<SIZE_T>(base + slot) * compute_->descriptor_size;
        device_->CreateShaderResourceView(resource, &description, handle);
    }

    void d3d12_renderer::write_texture_srv(std::uint32_t base, std::uint32_t slot,
        ID3D12Resource* resource, DXGI_FORMAT format)
    {
        if (resource == nullptr || slot >= compute_srv_count) return;
        D3D12_SHADER_RESOURCE_VIEW_DESC description{};
        description.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        description.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        description.Format = format;
        description.Texture2D.MipLevels = 1;
        description.Texture2D.MostDetailedMip = 0;
        D3D12_CPU_DESCRIPTOR_HANDLE handle = compute_->descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<SIZE_T>(base + slot) * compute_->descriptor_size;
        device_->CreateShaderResourceView(resource, &description, handle);
    }

    void d3d12_renderer::write_buffer_uav(std::uint32_t base, std::uint32_t slot,
        ID3D12Resource* resource, std::uint32_t element_count, std::uint32_t stride)
    {
        if (resource == nullptr || slot >= compute_uav_count) return;
        D3D12_UNORDERED_ACCESS_VIEW_DESC description{};
        description.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.Buffer.FirstElement = 0;
        description.Buffer.NumElements = element_count;
        description.Buffer.StructureByteStride = stride;
        D3D12_CPU_DESCRIPTOR_HANDLE handle = compute_->descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<SIZE_T>(base + compute_srv_count + slot) * compute_->descriptor_size;
        device_->CreateUnorderedAccessView(resource, nullptr, &description, handle);
    }

    void d3d12_renderer::dispatch_compute(ID3D12PipelineState* pipeline, std::uint32_t base,
        const void* constants, std::uint32_t constant_count, std::uint32_t group_count)
    {
        ID3D12DescriptorHeap* heaps[] = {compute_->descriptor_heap.Get()};
        command_list_->SetDescriptorHeaps(1, heaps);
        command_list_->SetComputeRootSignature(compute_->root_signature.Get());
        command_list_->SetPipelineState(pipeline);
        command_list_->SetComputeRoot32BitConstants(0, constant_count, constants, 0);

        D3D12_GPU_DESCRIPTOR_HANDLE handle = compute_->descriptor_heap->GetGPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<UINT64>(base) * compute_->descriptor_size;
        command_list_->SetComputeRootDescriptorTable(1, handle);
        command_list_->Dispatch(std::max(group_count, 1u), 1, 1);
    }

    void d3d12_renderer::transition_resource(ID3D12Resource* resource,
        D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after)
    {
        transition_barrier(command_list_.Get(), resource, before, after);
    }

    void d3d12_renderer::simulate_volume(float delta_time)
    {
        const float clamped_delta_time = std::clamp(delta_time, 1.0f / 240.0f, 1.0f / 30.0f);
        const std::uint32_t particle_groups = (compute_->particle_count + 255u) / 256u;
        const std::uint32_t tetrahedron_groups = (compute_->tetrahedron_count + 127u) / 128u;

        for (compute_state::gpu_body& body : compute_->bodies)
        {
            if (body.lambda_zero_is_srv)
            {
                transition_resource(body.tetrahedron_lambdas[0].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                body.lambda_zero_is_srv = false;
            }

            const std::uint32_t lambda_count = compute_->tetrahedron_count * 7;
            const std::uint32_t clear_table = reserve_descriptor_table();
            clear_descriptor_table(clear_table);
            write_buffer_uav(clear_table, 0, body.tetrahedron_lambdas[0].Get(),
                lambda_count, sizeof(float));
            const clear_constants clear_values{lambda_count, {0.0f, 0.0f, 0.0f}};
            dispatch_compute(compute_->clear_pipeline.Get(), clear_table, &clear_values, 4,
                (lambda_count + 255u) / 256u);
            transition_resource(body.tetrahedron_lambdas[0].Get(),
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            const std::uint32_t integration_output = 1u - body.particle_read;
            transition_resource(body.particles[integration_output].Get(),
                D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            const std::uint32_t integrate_table = reserve_descriptor_table();
            clear_descriptor_table(integrate_table);
            write_buffer_srv(integrate_table, 0, body.particles[body.particle_read].Get(),
                compute_->particle_count, sizeof(simulation_particle));
            write_buffer_uav(integrate_table, 0, body.particles[integration_output].Get(),
                compute_->particle_count, sizeof(simulation_particle));
            integrate_constants integration{};
            integration.particle_count = compute_->particle_count;
            integration.delta_time = clamped_delta_time;
            integration.damping = 0.45f;
            integration.gravity[0] = 0.0f;
            integration.gravity[1] = 0.0f;
            integration.gravity[2] = 0.0f;
            dispatch_compute(compute_->integrate_pipeline.Get(), integrate_table, &integration, 8, particle_groups);
            transition_resource(body.particles[integration_output].Get(),
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            body.particle_read = integration_output;

            std::uint32_t lambda_read = 0;
            for (std::uint32_t iteration = 0; iteration < simulation_parameters_.solver_iterations; ++iteration)
            {
                if (body.correction_is_srv)
                {
                    transition_resource(body.tetrahedron_corrections.Get(),
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                    body.correction_is_srv = false;
                }

                const std::uint32_t lambda_write = 1u - lambda_read;
                const std::uint32_t solve_table = reserve_descriptor_table();
                clear_descriptor_table(solve_table);
                write_buffer_srv(solve_table, 0, body.particles[body.particle_read].Get(),
                    compute_->particle_count, sizeof(simulation_particle));
                write_buffer_srv(solve_table, 1, compute_->tetrahedra.Get(),
                    compute_->tetrahedron_count, sizeof(tetrahedron));
                write_buffer_srv(solve_table, 2, compute_->inverse_masses.Get(),
                    compute_->particle_count, sizeof(float));
                write_buffer_srv(solve_table, 3, body.tetrahedron_lambdas[lambda_read].Get(),
                    compute_->tetrahedron_count * 7, sizeof(float));
                write_buffer_srv(solve_table, 4, compute_->rest_positions.Get(),
                    compute_->particle_count, sizeof(DirectX::XMFLOAT4));
                write_buffer_uav(solve_table, 0, body.tetrahedron_corrections.Get(),
                    compute_->tetrahedron_count * 4, sizeof(DirectX::XMFLOAT4));
                write_buffer_uav(solve_table, 1, body.tetrahedron_lambdas[lambda_write].Get(),
                    compute_->tetrahedron_count * 7, sizeof(float));
                const tetrahedron_constants solve_constants{
                    compute_->tetrahedron_count, clamped_delta_time, 0.65f, 0.0f
                };
                dispatch_compute(compute_->tetrahedron_pipeline.Get(), solve_table,
                    &solve_constants, 4, tetrahedron_groups);

                transition_resource(body.tetrahedron_corrections.Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.correction_is_srv = true;
                transition_resource(body.tetrahedron_lambdas[lambda_write].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                transition_resource(body.tetrahedron_lambdas[lambda_read].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

                const std::uint32_t particle_output = 1u - body.particle_read;
                transition_resource(body.particles[particle_output].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                const std::uint32_t gather_table = reserve_descriptor_table();
                clear_descriptor_table(gather_table);
                write_buffer_srv(gather_table, 0, body.particles[body.particle_read].Get(),
                    compute_->particle_count, sizeof(simulation_particle));
                write_buffer_srv(gather_table, 1, compute_->incidence_ranges.Get(),
                    compute_->particle_count, sizeof(uint2));
                write_buffer_srv(gather_table, 2, compute_->incident_tetrahedra.Get(),
                    static_cast<std::uint32_t>(compute_->tetrahedron_count * 4), sizeof(std::uint32_t));
                write_buffer_srv(gather_table, 3, body.tetrahedron_corrections.Get(),
                    compute_->tetrahedron_count * 4, sizeof(DirectX::XMFLOAT4));
                write_buffer_srv(gather_table, 4, compute_->tetrahedron_indices.Get(),
                    compute_->tetrahedron_count, sizeof(std::array<std::uint32_t, 4>));
                write_buffer_uav(gather_table, 0, body.particles[particle_output].Get(),
                    compute_->particle_count, sizeof(simulation_particle));
                const gather_constants gather_values{
                    compute_->particle_count, 0.025f, clamped_delta_time, 0.0f
                };
                dispatch_compute(compute_->gather_pipeline.Get(), gather_table,
                    &gather_values, 4, particle_groups);
                transition_resource(body.particles[particle_output].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.particle_read = particle_output;
                lambda_read = lambda_write;
            }
            body.lambda_zero_is_srv = lambda_read == 0;
        }
    }

    void d3d12_renderer::update_skeletal_pose(float delta_time)
    {
        if (compute_->animation.frames.empty() || compute_->animation.bones.empty()
            || compute_->skeletal_vertex_weights.size() != compute_->vertex_count
            || compute_->mapped_surface_rest_positions == nullptr
            || compute_->mapped_surface_edges == nullptr
            || compute_->mapped_surface_triangle_areas == nullptr
            || compute_->mapped_surface_bending == nullptr)
            return;

        compute_->animation_time += std::clamp(delta_time, 1.0f / 240.0f, 1.0f / 30.0f);
        sample_skin_matrices(compute_->animation, compute_->animation_time,
            compute_->sampled_skin_matrices);

        for (std::uint32_t vertex_index = 0; vertex_index < compute_->vertex_count; ++vertex_index)
        {
            const std::array<float, 3> position = skin_vertex_position(
                compute_->skeletal_rest_positions[vertex_index],
                compute_->skeletal_vertex_weights[vertex_index],
                compute_->sampled_skin_matrices);
            compute_->mapped_surface_rest_positions[vertex_index] =
                DirectX::XMFLOAT4(position[0], position[1], position[2], 1.0f);
        }

        const auto position_at = [this](std::uint32_t index) -> const DirectX::XMFLOAT4&
        {
            return compute_->mapped_surface_rest_positions[index];
        };
        for (std::uint32_t edge_index = 0; edge_index < compute_->edge_count; ++edge_index)
        {
            surface_edge& edge = compute_->mapped_surface_edges[edge_index];
            const DirectX::XMFLOAT4& a = position_at(edge.indices[0]);
            const DirectX::XMFLOAT4& b = position_at(edge.indices[1]);
            const float x = b.x - a.x;
            const float y = b.y - a.y;
            const float z = b.z - a.z;
            edge.rest_length = std::sqrt(x * x + y * y + z * z);
        }

        for (std::uint32_t triangle_index = 0; triangle_index < compute_->triangle_count; ++triangle_index)
        {
            triangle_area_constraint& triangle = compute_->mapped_surface_triangle_areas[triangle_index];
            const DirectX::XMFLOAT4& a = position_at(triangle.indices[0]);
            const DirectX::XMFLOAT4& b = position_at(triangle.indices[1]);
            const DirectX::XMFLOAT4& c = position_at(triangle.indices[2]);
            const float ab_x = b.x - a.x;
            const float ab_y = b.y - a.y;
            const float ab_z = b.z - a.z;
            const float ac_x = c.x - a.x;
            const float ac_y = c.y - a.y;
            const float ac_z = c.z - a.z;
            const float cross_x = ab_y * ac_z - ab_z * ac_y;
            const float cross_y = ab_z * ac_x - ab_x * ac_z;
            const float cross_z = ab_x * ac_y - ab_y * ac_x;
            triangle.rest_area = 0.5f * std::sqrt(
                cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
        }

        for (std::uint32_t hinge_index = 0; hinge_index < compute_->bending_count; ++hinge_index)
        {
            bending_constraint& hinge = compute_->mapped_surface_bending[hinge_index];
            const DirectX::XMFLOAT4& a = position_at(hinge.indices[2]);
            const DirectX::XMFLOAT4& b = position_at(hinge.indices[3]);
            const float x = b.x - a.x;
            const float y = b.y - a.y;
            const float z = b.z - a.z;
            hinge.rest_distance = std::sqrt(x * x + y * y + z * z);
        }
    }

    void d3d12_renderer::simulate_camera_surface(float delta_time)
    {
        const float clamped_delta_time = std::clamp(delta_time, 1.0f / 240.0f, 1.0f / 30.0f);
        const std::uint32_t particle_groups = (compute_->vertex_count + 255u) / 256u;
        const std::uint32_t edge_groups = (compute_->edge_count + 255u) / 256u;
        const std::uint32_t triangle_groups = (compute_->triangle_count + 255u) / 256u;
        const std::uint32_t bending_groups = (compute_->bending_count + 255u) / 256u;

        std::uint32_t camera_dispatch_index = 0;
        for (compute_state::camera_gpu_body& body : compute_->camera_bodies)
        {
            if (body.lambda_zero_is_srv)
            {
                transition_resource(body.edge_lambdas[0].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                body.lambda_zero_is_srv = false;
            }

            if (body.area_lambda_zero_is_srv)
            {
                transition_resource(body.area_lambdas[0].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                body.area_lambda_zero_is_srv = false;
            }

            if (compute_->bending_count > 0 && body.bending_lambda_zero_is_srv)
            {
                transition_resource(body.bending_lambdas[0].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                body.bending_lambda_zero_is_srv = false;
            }

            const std::uint32_t clear_table = reserve_descriptor_table();
            clear_descriptor_table(clear_table);
            write_buffer_uav(clear_table, 0, body.edge_lambdas[0].Get(),
                compute_->edge_count, sizeof(float));
            const clear_constants edge_clear_values{compute_->edge_count, {0.0f, 0.0f, 0.0f}};
            dispatch_compute(compute_->clear_pipeline.Get(), clear_table, &edge_clear_values, 4,
                (compute_->edge_count + 255u) / 256u);
            transition_resource(body.edge_lambdas[0].Get(),
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            const std::uint32_t area_clear_table = reserve_descriptor_table();
            clear_descriptor_table(area_clear_table);
            write_buffer_uav(area_clear_table, 0, body.area_lambdas[0].Get(),
                compute_->triangle_count, sizeof(float));
            const clear_constants area_clear_values{compute_->triangle_count, {0.0f, 0.0f, 0.0f}};
            dispatch_compute(compute_->clear_pipeline.Get(), area_clear_table, &area_clear_values, 4,
                triangle_groups);
            transition_resource(body.area_lambdas[0].Get(),
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            if (compute_->bending_count > 0)
            {
                const std::uint32_t bending_clear_table = reserve_descriptor_table();
                clear_descriptor_table(bending_clear_table);
                write_buffer_uav(bending_clear_table, 0, body.bending_lambdas[0].Get(),
                    compute_->bending_count, sizeof(float));
                const clear_constants bending_clear_values{compute_->bending_count, {0.0f, 0.0f, 0.0f}};
                dispatch_compute(compute_->clear_pipeline.Get(), bending_clear_table, &bending_clear_values, 4,
                    bending_groups);
                transition_resource(body.bending_lambdas[0].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            }

            const std::uint32_t integration_output = 1u - body.particle_read;
            transition_resource(body.particles[integration_output].Get(),
                D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            const std::uint32_t integrate_table = reserve_descriptor_table();
            clear_descriptor_table(integrate_table);
            write_buffer_srv(integrate_table, 0, body.particles[body.particle_read].Get(),
                compute_->vertex_count, sizeof(simulation_particle));
            write_buffer_uav(integrate_table, 0, body.particles[integration_output].Get(),
                compute_->vertex_count, sizeof(simulation_particle));

            integrate_constants integration{};
            integration.particle_count = compute_->vertex_count;
            integration.delta_time = clamped_delta_time;
            integration.damping = 0.45f;
            dispatch_compute(compute_->integrate_pipeline.Get(), integrate_table,
                &integration, 8, particle_groups);
            transition_resource(body.particles[integration_output].Get(),
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            body.particle_read = integration_output;

            std::uint32_t edge_lambda_read = 0;
            std::uint32_t area_lambda_read = 0;
            std::uint32_t bending_lambda_read = 0;
            for (std::uint32_t iteration = 0; iteration < simulation_parameters_.solver_iterations; ++iteration)
            {
                if (body.correction_is_srv)
                {
                    transition_resource(body.edge_corrections.Get(),
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                    body.correction_is_srv = false;
                }

                const std::uint32_t edge_lambda_write = 1u - edge_lambda_read;
                const std::uint32_t solve_table = reserve_descriptor_table();
                clear_descriptor_table(solve_table);
                write_buffer_srv(solve_table, 0, body.particles[body.particle_read].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                write_buffer_srv(solve_table, 1, compute_->surface_edges.Get(),
                    compute_->edge_count, sizeof(surface_edge));
                write_buffer_srv(solve_table, 2, body.edge_lambdas[edge_lambda_read].Get(),
                    compute_->edge_count, sizeof(float));
                write_buffer_uav(solve_table, 0, body.edge_corrections.Get(),
                    compute_->edge_count + compute_->triangle_count * 3u, sizeof(DirectX::XMFLOAT3));
                write_buffer_uav(solve_table, 1, body.edge_lambdas[edge_lambda_write].Get(),
                    compute_->edge_count, sizeof(float));

                const tetrahedron_constants edge_constants{
                    compute_->edge_count, clamped_delta_time, simulation_parameters_.edge_relaxation,
                    simulation_parameters_.edge_compliance
                };
                const std::uint32_t query_base = 5u + camera_dispatch_index * 8u;
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base);
                dispatch_compute(compute_->camera_surface_edge_pipeline.Get(), solve_table,
                    &edge_constants, 4, edge_groups);
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 1u);

                const std::uint32_t area_lambda_write = 1u - area_lambda_read;
                const std::uint32_t area_table = reserve_descriptor_table();
                clear_descriptor_table(area_table);
                write_buffer_srv(area_table, 0, body.particles[body.particle_read].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                write_buffer_srv(area_table, 1, compute_->surface_triangle_areas.Get(),
                    compute_->triangle_count, sizeof(triangle_area_constraint));
                write_buffer_srv(area_table, 2, body.area_lambdas[area_lambda_read].Get(),
                    compute_->triangle_count, sizeof(float));
                write_buffer_uav(area_table, 0, body.edge_corrections.Get(),
                    compute_->edge_count + compute_->triangle_count * 3u, sizeof(DirectX::XMFLOAT3));
                write_buffer_uav(area_table, 1, body.area_lambdas[area_lambda_write].Get(),
                    compute_->triangle_count, sizeof(float));
                const camera_area_constants area_constants{
                    compute_->triangle_count, compute_->edge_count,
                    clamped_delta_time, simulation_parameters_.edge_relaxation,
                    simulation_parameters_.area_compliance
                };
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 2u);
                dispatch_compute(compute_->camera_surface_area_pipeline.Get(), area_table,
                    &area_constants, 5, triangle_groups);
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 3u);
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 4u);

                std::uint32_t bending_lambda_write = bending_lambda_read;
                if (compute_->bending_count > 0)
                {
                    bending_lambda_write = 1u - bending_lambda_read;
                    const std::uint32_t bending_table = reserve_descriptor_table();
                    clear_descriptor_table(bending_table);
                    write_buffer_srv(bending_table, 0, body.particles[body.particle_read].Get(),
                        compute_->vertex_count, sizeof(simulation_particle));
                    write_buffer_srv(bending_table, 1, compute_->surface_bending.Get(),
                        compute_->bending_count, sizeof(bending_constraint));
                    write_buffer_srv(bending_table, 2, body.bending_lambdas[bending_lambda_read].Get(),
                        compute_->bending_count, sizeof(float));
                    write_buffer_uav(bending_table, 0, body.edge_corrections.Get(),
                        compute_->edge_count + compute_->triangle_count * 3u + compute_->bending_count * 4u,
                        sizeof(DirectX::XMFLOAT3));
                    write_buffer_uav(bending_table, 1, body.bending_lambdas[bending_lambda_write].Get(),
                        compute_->bending_count, sizeof(float));
                    const camera_bending_constants bending_constants{
                        compute_->bending_count, compute_->edge_count, compute_->triangle_count,
                        clamped_delta_time, simulation_parameters_.edge_relaxation,
                        simulation_parameters_.bending_compliance, {0.0f, 0.0f}
                    };
                    dispatch_compute(compute_->camera_surface_bending_pipeline.Get(), bending_table,
                        &bending_constants, 8, bending_groups);
    
                }
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 5u);

                transition_resource(body.edge_corrections.Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.correction_is_srv = true;
                transition_resource(body.edge_lambdas[edge_lambda_write].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                transition_resource(body.edge_lambdas[edge_lambda_read].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                transition_resource(body.area_lambdas[area_lambda_write].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                transition_resource(body.area_lambdas[area_lambda_read].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                if (compute_->bending_count > 0)
                {
                    transition_resource(body.bending_lambdas[bending_lambda_write].Get(),
                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                    transition_resource(body.bending_lambdas[bending_lambda_read].Get(),
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                }

                const std::uint32_t particle_output = 1u - body.particle_read;
                transition_resource(body.particles[particle_output].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                const std::uint32_t gather_table = reserve_descriptor_table();
                clear_descriptor_table(gather_table);
                write_buffer_srv(gather_table, 0, body.particles[body.particle_read].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                write_buffer_srv(gather_table, 1, compute_->surface_edge_ranges.Get(),
                    compute_->vertex_count, sizeof(mesh_uint2));
                write_buffer_srv(gather_table, 2, compute_->incident_surface_corrections.Get(),
                    std::max(1u, compute_->edge_count * 2u), sizeof(std::uint32_t));
                write_buffer_srv(gather_table, 3, body.edge_corrections.Get(),
                    compute_->edge_count + compute_->triangle_count * 3u + compute_->bending_count * 4u,
                    sizeof(DirectX::XMFLOAT3));
                write_buffer_srv(gather_table, 4, compute_->surface_triangle_ranges.Get(),
                    compute_->vertex_count, sizeof(mesh_uint2));
                write_buffer_srv(gather_table, 5, compute_->incident_surface_triangles.Get(),
                    std::max(1u, compute_->triangle_count * 3u), sizeof(std::uint32_t));
                write_buffer_srv(gather_table, 6, compute_->surface_bending_ranges.Get(),
                    compute_->vertex_count, sizeof(mesh_uint2));
                write_buffer_srv(gather_table, 7, compute_->incident_surface_bending.Get(),
                    std::max(1u, compute_->bending_count * 4u), sizeof(std::uint32_t));
                write_buffer_uav(gather_table, 0, body.particles[particle_output].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));

                const camera_gather_constants gather_values{
                    compute_->vertex_count, simulation_parameters_.max_correction,
                    clamped_delta_time, compute_->edge_count, compute_->triangle_count, {0u, 0u, 0u}
                };
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 6u);
                dispatch_compute(compute_->camera_surface_gather_pipeline.Get(), gather_table,
                    &gather_values, 5, particle_groups);
                command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, query_base + 7u);
                ++camera_dispatch_index;
                transition_resource(body.particles[particle_output].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.particle_read = particle_output;

                if (body.shape_transforms_is_srv)
                {
                    transition_resource(body.shape_cluster_transforms.Get(),
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                    body.shape_transforms_is_srv = false;
                }

                const std::uint32_t shape_build_table = reserve_descriptor_table();
                clear_descriptor_table(shape_build_table);
                write_buffer_srv(shape_build_table, 0, body.particles[body.particle_read].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                write_buffer_srv(shape_build_table, 1, compute_->surface_rest_positions.Get(),
                    compute_->vertex_count, sizeof(DirectX::XMFLOAT4));
                write_buffer_srv(shape_build_table, 2, compute_->shape_cluster_ranges.Get(),
                    compute_->shape_cluster_count, sizeof(mesh_uint2));
                write_buffer_srv(shape_build_table, 3, compute_->shape_cluster_vertices.Get(),
                    compute_->shape_membership_count, sizeof(std::uint32_t));
                write_buffer_uav(shape_build_table, 0, body.shape_cluster_transforms.Get(),
                    compute_->shape_cluster_count, sizeof(DirectX::XMFLOAT4) * 3u);
                const shape_build_constants shape_build_values{
                    compute_->shape_cluster_count, {0u, 0u, 0u}
                };
                dispatch_compute(compute_->camera_surface_shape_build_pipeline.Get(),
                    shape_build_table, &shape_build_values, 4,
                    (compute_->shape_cluster_count + 63u) / 64u);
                transition_resource(body.shape_cluster_transforms.Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.shape_transforms_is_srv = true;

                const std::uint32_t shape_output = 1u - body.particle_read;
                transition_resource(body.particles[shape_output].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                const std::uint32_t shape_apply_table = reserve_descriptor_table();
                clear_descriptor_table(shape_apply_table);
                write_buffer_srv(shape_apply_table, 0, body.particles[body.particle_read].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                write_buffer_srv(shape_apply_table, 1, compute_->surface_rest_positions.Get(),
                    compute_->vertex_count, sizeof(DirectX::XMFLOAT4));
                write_buffer_srv(shape_apply_table, 4, compute_->vertex_cluster_ranges.Get(),
                    compute_->vertex_count, sizeof(mesh_uint2));
                write_buffer_srv(shape_apply_table, 5, compute_->vertex_clusters.Get(),
                    compute_->shape_membership_count, sizeof(std::uint32_t));
                write_buffer_srv(shape_apply_table, 6, body.shape_cluster_transforms.Get(),
                    compute_->shape_cluster_count, sizeof(DirectX::XMFLOAT4) * 3u);
                write_buffer_uav(shape_apply_table, 1, body.particles[shape_output].Get(),
                    compute_->vertex_count, sizeof(simulation_particle));
                const shape_apply_constants shape_apply_values{
                    compute_->vertex_count, simulation_parameters_.shape_matching_strength,
                    compute_->animation.frames.empty() ? 0.0f : simulation_parameters_.bone_follow_strength,
                    simulation_parameters_.max_correction, clamped_delta_time,
                    body.center_x, {0.0f, 0.0f}
                };
                dispatch_compute(compute_->camera_surface_shape_apply_pipeline.Get(),
                    shape_apply_table, &shape_apply_values, 8, particle_groups);
                transition_resource(body.particles[shape_output].Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                body.particle_read = shape_output;

                edge_lambda_read = edge_lambda_write;
                area_lambda_read = area_lambda_write;
                if (compute_->bending_count > 0) bending_lambda_read = bending_lambda_write;
            }
            body.lambda_zero_is_srv = edge_lambda_read == 0;
            body.area_lambda_zero_is_srv = area_lambda_read == 0;
            body.bending_lambda_zero_is_srv = compute_->bending_count > 0 && bending_lambda_read == 0;
        }
    }

    void d3d12_renderer::skin_camera_body(std::uint32_t body_index)
    {
        compute_state::camera_gpu_body& body = compute_->camera_bodies[body_index];
        if (body.vertex_is_input_buffer)
        {
            transition_resource(body.vertices.Get(),
                D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            body.vertex_is_input_buffer = false;
        }

        const std::uint32_t table = reserve_descriptor_table();
        clear_descriptor_table(table);
        write_buffer_srv(table, 0, body.particles[body.particle_read].Get(),
            compute_->vertex_count, sizeof(simulation_particle));
        write_buffer_srv(table, 1, compute_->surface_triangle_ranges.Get(),
            compute_->vertex_count, sizeof(mesh_uint2));
        write_buffer_srv(table, 2, compute_->incident_surface_triangles.Get(),
            compute_->triangle_count * 3u, sizeof(std::uint32_t));
        write_buffer_srv(table, 3, compute_->surface_triangle_areas.Get(),
            compute_->triangle_count, sizeof(triangle_area_constraint));
        write_buffer_uav(table, 0, body.vertices.Get(), compute_->vertex_count, sizeof(vertex));
        const skin_constants constants{
            compute_->vertex_count, {body.center_x, 0.0f, 0.0f}
        };
        dispatch_compute(compute_->camera_surface_skin_pipeline.Get(), table, &constants, 4,
            (compute_->vertex_count + 255u) / 256u);

        transition_resource(body.vertices.Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
        body.vertex_is_input_buffer = true;
    }

    void d3d12_renderer::skin_body(std::uint32_t body_index)
    {
        compute_state::gpu_body& body = compute_->bodies[body_index];
        if (body.vertex_is_input_buffer)
        {
            transition_resource(body.vertices.Get(),
                D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            body.vertex_is_input_buffer = false;
        }

        const std::uint32_t table = reserve_descriptor_table();
        clear_descriptor_table(table);
        write_buffer_srv(table, 0, body.particles[body.particle_read].Get(),
            compute_->particle_count, sizeof(simulation_particle));
        write_buffer_srv(table, 1, compute_->surface_mappings.Get(),
            compute_->vertex_count, sizeof(surface_mapping));
        write_buffer_uav(table, 0, body.vertices.Get(), compute_->vertex_count, sizeof(vertex));

        const skin_constants constants{
            compute_->vertex_count, {body.center_x, 0.0f, 0.0f}
        };
        dispatch_compute(compute_->skin_pipeline.Get(), table, &constants, 4,
            (compute_->vertex_count + 255u) / 256u);
        transition_resource(body.vertices.Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
        body.vertex_is_input_buffer = true;
    }

    void d3d12_renderer::capture_body(std::uint32_t capture_index, std::uint32_t body_index)
    {
        const D3D12_VERTEX_BUFFER_VIEW& vertex_view =
            solver_mode_ == solver_mode::camera_surface
                ? compute_->camera_bodies[body_index].vertex_view
                : compute_->bodies[body_index].vertex_view;
        D3D12_CPU_DESCRIPTOR_HANDLE rtv = capture_rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += static_cast<SIZE_T>(capture_index) * capture_rtv_descriptor_size_;
        D3D12_CPU_DESCRIPTOR_HANDLE dsv = capture_dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        dsv.ptr += static_cast<SIZE_T>(capture_index) * capture_dsv_descriptor_size_;
        command_list_->OMSetRenderTargets(1, &rtv, FALSE, &dsv);

        constexpr float clear_normal[4] = {0.5f, 0.5f, 1.0f, 1.0f};
        command_list_->ClearRenderTargetView(rtv, clear_normal, 0, nullptr);
        command_list_->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

        D3D12_VIEWPORT viewport{};
        viewport.Width = static_cast<float>(capture_size);
        viewport.Height = static_cast<float>(capture_size);
        viewport.MaxDepth = 1.0f;
        const D3D12_RECT scissor{0, 0, static_cast<LONG>(capture_size), static_cast<LONG>(capture_size)};
        command_list_->RSSetViewports(1, &viewport);
        command_list_->RSSetScissorRects(1, &scissor);
        command_list_->SetGraphicsRootSignature(capture_root_signature_.Get());
        command_list_->SetPipelineState(capture_pipeline_.Get());
        command_list_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        command_list_->IASetVertexBuffers(0, 1, &vertex_view);
        command_list_->IASetIndexBuffer(&index_buffer_view_);

        using namespace DirectX;
        const XMVECTOR capture_eye = capture_index == 0
            ? XMVectorSet(-6.0f, 0.0f, 0.0f, 1.0f)
            : XMVectorSet(6.0f, 0.0f, 0.0f, 1.0f);
        const XMMATRIX view = XMMatrixLookAtLH(capture_eye, XMVectorZero(),
            XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
        const XMMATRIX projection = XMMatrixOrthographicLH(4.8f, 4.8f, 0.1f, 20.0f);
        capture_constants constants{};
        XMStoreFloat4x4(&constants.world, XMMatrixIdentity());
        XMStoreFloat4x4(&constants.view_projection, view * projection);
        command_list_->SetGraphicsRoot32BitConstants(0, 32, &constants, 0);
        command_list_->DrawIndexedInstanced(index_count_, 1, 0, 0, 0);

        transition_resource(capture_normal_[capture_index].Get(),
            D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        transition_resource(capture_depth_[capture_index].Get(),
            D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    }

    void d3d12_renderer::resolve_benchmark_queries(double cpu_frame_ms)
    {
        if (!compute_ || timestamp_frequency_ == 0) return;
        const std::size_t mode_index =
            solver_mode_ == solver_mode::camera_surface ? 1u : 0u;
        if (!compute_->logs[mode_index]) return;

        const D3D12_RANGE read_range{0, sizeof(std::uint64_t) * (5u + max_solver_iterations * 2u * 8u)};
        std::uint64_t* timestamps = nullptr;
        if (FAILED(timestamp_readback_->Map(0, &read_range, reinterpret_cast<void**>(&timestamps)))) return;

        const std::uint64_t simulation_ticks = timestamps[1] - timestamps[0];
        const std::uint64_t capture_ticks = timestamps[2] - timestamps[1];
        const std::uint64_t collision_ticks = timestamps[3] - timestamps[2];
        const std::uint64_t render_ticks = timestamps[4] - timestamps[3];
        std::uint64_t camera_edge_ticks = 0;
        std::uint64_t camera_area_ticks = 0;
        std::uint64_t camera_bending_ticks = 0;
        std::uint64_t camera_gather_ticks = 0;
        if (mode_index == 1u && !simulation_paused_)
        {
            for (std::uint32_t dispatch_index = 0; dispatch_index < simulation_parameters_.solver_iterations * 2u; ++dispatch_index)
            {
                const std::uint32_t query_base = 5u + dispatch_index * 8u;
                camera_edge_ticks += timestamps[query_base + 1u] - timestamps[query_base];
                camera_area_ticks += timestamps[query_base + 3u] - timestamps[query_base + 2u];
                camera_bending_ticks += timestamps[query_base + 5u] - timestamps[query_base + 4u];
                camera_gather_ticks += timestamps[query_base + 7u] - timestamps[query_base + 6u];
            }
        }
        const double ticks_to_ms = 1000.0 / static_cast<double>(timestamp_frequency_);
        performance_metrics_.cpu_frame_ms = cpu_frame_ms;
        performance_metrics_.gpu_simulation_ms = static_cast<double>(simulation_ticks) * ticks_to_ms;
        performance_metrics_.camera_edge_solve_ms = static_cast<double>(camera_edge_ticks) * ticks_to_ms;
        performance_metrics_.camera_area_solve_ms = static_cast<double>(camera_area_ticks) * ticks_to_ms;
        performance_metrics_.camera_bending_solve_ms = static_cast<double>(camera_bending_ticks) * ticks_to_ms;
        performance_metrics_.camera_constraint_solve_ms = performance_metrics_.camera_edge_solve_ms
            + performance_metrics_.camera_area_solve_ms + performance_metrics_.camera_bending_solve_ms;
        performance_metrics_.camera_gather_ms = static_cast<double>(camera_gather_ticks) * ticks_to_ms;
        performance_metrics_.gpu_depth_capture_ms = static_cast<double>(capture_ticks) * ticks_to_ms;
        performance_metrics_.gpu_collision_ms = static_cast<double>(collision_ticks) * ticks_to_ms;
        performance_metrics_.gpu_render_ms = static_cast<double>(render_ticks) * ticks_to_ms;
        performance_metrics_.gpu_total_ms = performance_metrics_.gpu_simulation_ms
            + performance_metrics_.gpu_depth_capture_ms + performance_metrics_.gpu_collision_ms
            + performance_metrics_.gpu_render_ms;
        const std::uint64_t particle_count =
            (mode_index == 1u ? compute_->vertex_count : compute_->particle_count) * 2ull;
        const std::uint64_t tetrahedron_count =
            mode_index == 1u ? 0ull : compute_->tetrahedron_count * 2ull;

        compute_->logs[mode_index]->write_frame(++compute_->benchmark_frame_numbers[mode_index],
            cpu_frame_ms,
            static_cast<double>(simulation_ticks) * ticks_to_ms,
            static_cast<double>(camera_edge_ticks) * ticks_to_ms,
            static_cast<double>(camera_area_ticks) * ticks_to_ms,
            static_cast<double>(camera_bending_ticks) * ticks_to_ms,
            static_cast<double>(camera_gather_ticks) * ticks_to_ms,
            static_cast<double>(capture_ticks) * ticks_to_ms,
            static_cast<double>(collision_ticks) * ticks_to_ms,
            static_cast<double>(render_ticks) * ticks_to_ms,
            particle_count, tetrahedron_count,
            mode_index == 1u ? compute_->camera_surface_buffer_bytes : compute_->tetrahedral_buffer_bytes,
            compute_->resident_simulation_buffer_bytes,
            simulation_parameters_.solver_iterations, simulation_parameters_.edge_relaxation,
            simulation_parameters_.max_correction, simulation_parameters_.particle_radius,
            simulation_parameters_.pushout, simulation_parameters_.velocity_friction,
            simulation_parameters_.restitution, simulation_parameters_.contact_band,
            simulation_parameters_.shape_matching_strength);

        const D3D12_RANGE written_range{0, 0};
        timestamp_readback_->Unmap(0, &written_range);
    }

    void d3d12_renderer::render(float delta_time)
    {
        const std::chrono::steady_clock::time_point cpu_start = std::chrono::steady_clock::now();
        compute_->descriptor_cursor = 0;
        const bool use_camera_surface = solver_mode_ == solver_mode::camera_surface;

        ID3D12CommandAllocator* allocator = command_allocators_[frame_index_].Get();
        if (FAILED(allocator->Reset())) throw std::runtime_error("Command allocator reset failed.");
        if (FAILED(command_list_->Reset(allocator, graphics_pipeline_.Get())))
            throw std::runtime_error("Command list reset failed.");

        command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 0);
        if (!simulation_paused_)
        {
            if (use_camera_surface)
            {
                update_skeletal_pose(delta_time);
                simulate_camera_surface(delta_time);
                skin_camera_body(0);
                skin_camera_body(1);
            }
            else
            {
                simulate_volume(delta_time);
                skin_body(0);
                skin_body(1);
            }
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 1);

            capture_body(0, 1);
            capture_body(1, 0);
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 2);

            using namespace DirectX;
            for (std::uint32_t body_index = 0; body_index < 2; ++body_index)
            {
                const XMVECTOR collision_eye = body_index == 0
                    ? XMVectorSet(-6.0f, 0.0f, 0.0f, 1.0f)
                    : XMVectorSet(6.0f, 0.0f, 0.0f, 1.0f);
                const XMMATRIX collision_view = XMMatrixLookAtLH(collision_eye, XMVectorZero(),
                    XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
                const XMMATRIX collision_projection = XMMatrixOrthographicLH(4.8f, 4.8f, 0.1f, 20.0f);
                const XMMATRIX collision_view_projection = collision_view * collision_projection;
                XMVECTOR determinant{};
                const XMMATRIX inverse_view_projection = XMMatrixInverse(&determinant, collision_view_projection);

                camera_collision_constants collision_constants{};
                XMStoreFloat4x4(&collision_constants.view_projection, collision_view_projection);
                XMStoreFloat4x4(&collision_constants.inverse_view_projection, inverse_view_projection);
                collision_constants.particle_count =
                    use_camera_surface ? compute_->vertex_count : compute_->particle_count;
                collision_constants.viewport_width = capture_size;
                collision_constants.viewport_height = capture_size;
                collision_constants.particle_radius = use_camera_surface ? simulation_parameters_.particle_radius : 0.035f;
                collision_constants.pushout = use_camera_surface ? simulation_parameters_.pushout : 0.012f;
                collision_constants.velocity_friction = use_camera_surface ? simulation_parameters_.velocity_friction : 0.12f;
                collision_constants.restitution = use_camera_surface ? simulation_parameters_.restitution : 0.18f;
                collision_constants.contact_band = use_camera_surface ? simulation_parameters_.contact_band : 0.16f;
                collision_constants.smooth_surface_contacts = use_camera_surface ? 1u : 0u;

                ID3D12Resource* input_particles = nullptr;
                ID3D12Resource* output_particles = nullptr;
                std::uint32_t* particle_read = nullptr;
                if (use_camera_surface)
                {
                    compute_state::camera_gpu_body& body = compute_->camera_bodies[body_index];
                    particle_read = &body.particle_read;
                    input_particles = body.particles[body.particle_read].Get();
                    output_particles = body.particles[1u - body.particle_read].Get();
                }
                else
                {
                    compute_state::gpu_body& body = compute_->bodies[body_index];
                    particle_read = &body.particle_read;
                    input_particles = body.particles[body.particle_read].Get();
                    output_particles = body.particles[1u - body.particle_read].Get();
                }

                transition_resource(output_particles,
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                const std::uint32_t table = reserve_descriptor_table();
                clear_descriptor_table(table);
                write_texture_srv(table, 0, capture_depth_[body_index].Get(), DXGI_FORMAT_R32_FLOAT);
                write_texture_srv(table, 1, capture_normal_[body_index].Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
                write_buffer_srv(table, 2, input_particles,
                    collision_constants.particle_count, sizeof(simulation_particle));
                write_buffer_uav(table, 0, output_particles,
                    collision_constants.particle_count, sizeof(simulation_particle));
                std::uint32_t contact_write_index = 0;
                if (use_camera_surface)
                {
                    compute_state::camera_gpu_body& body = compute_->camera_bodies[body_index];
                    contact_write_index = 1u - body.contact_correction_read;
                    if (body.contact_correction_is_srv[contact_write_index])
                    {
                        transition_resource(body.contact_corrections[contact_write_index].Get(),
                            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                            D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
                        body.contact_correction_is_srv[contact_write_index] = false;
                    }
                    write_buffer_uav(table, 1, body.contact_corrections[contact_write_index].Get(),
                        compute_->vertex_count, sizeof(DirectX::XMFLOAT4));
                }
                dispatch_compute(compute_->camera_collision_pipeline.Get(), table,
                    &collision_constants, 41,
                    (collision_constants.particle_count + 255u) / 256u);
                transition_resource(output_particles,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                *particle_read = 1u - *particle_read;

                if (use_camera_surface)
                {
                    compute_state::camera_gpu_body& body = compute_->camera_bodies[body_index];
                    ID3D12Resource* correction_input = body.contact_corrections[contact_write_index].Get();
                    transition_resource(correction_input, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                    body.contact_correction_is_srv[contact_write_index] = true;

                    constexpr float contact_smoothing_strength = 0.85f;
                    const std::uint32_t apply_output_index = 1u - *particle_read;
                    ID3D12Resource* apply_output = body.particles[apply_output_index].Get();
                    transition_resource(apply_output, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

                    const std::uint32_t apply_table = reserve_descriptor_table();
                    clear_descriptor_table(apply_table);
                    write_buffer_srv(apply_table, 0, correction_input,
                        compute_->vertex_count, sizeof(DirectX::XMFLOAT4));
                    write_buffer_srv(apply_table, 1, compute_->surface_edge_ranges.Get(),
                        compute_->vertex_count, sizeof(mesh_uint2));
                    write_buffer_srv(apply_table, 2, compute_->incident_surface_corrections.Get(),
                        compute_->edge_count * 2u, sizeof(std::uint32_t));
                    write_buffer_srv(apply_table, 3, compute_->surface_edges.Get(),
                        compute_->edge_count, sizeof(surface_edge));
                    write_buffer_srv(apply_table, 4, body.particles[*particle_read].Get(),
                        compute_->vertex_count, sizeof(simulation_particle));
                    write_buffer_uav(apply_table, 1, apply_output,
                        compute_->vertex_count, sizeof(simulation_particle));
                    const std::uint32_t apply_values[4] = {
                        compute_->vertex_count, std::bit_cast<std::uint32_t>(contact_smoothing_strength), 0u, 0u
                    };
                    dispatch_compute(compute_->camera_contact_apply_pipeline.Get(),
                        apply_table, apply_values, 4,
                        (compute_->vertex_count + 255u) / 256u);
                    transition_resource(apply_output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
                    *particle_read = apply_output_index;
                    body.contact_correction_read = contact_write_index;
                }
            }

            for (std::uint32_t capture_index = 0; capture_index < capture_count; ++capture_index)
            {
                transition_resource(capture_normal_[capture_index].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
                transition_resource(capture_depth_[capture_index].Get(),
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
            }

            if (use_camera_surface)
            {
                skin_camera_body(0);
                skin_camera_body(1);
            }
            else
            {
                skin_body(0);
                skin_body(1);
            }
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 3);
        }
        else
        {
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 1);
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 2);
            command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 3);
        }

        D3D12_RESOURCE_BARRIER back_buffer_barrier{};
        back_buffer_barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        back_buffer_barrier.Transition.pResource = render_targets_[frame_index_].Get();
        back_buffer_barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        back_buffer_barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        back_buffer_barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        command_list_->ResourceBarrier(1, &back_buffer_barrier);

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
        command_list_->SetPipelineState(graphics_pipeline_.Get());
        command_list_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        command_list_->IASetIndexBuffer(&index_buffer_view_);

        using namespace DirectX;
        const XMMATRIX view = XMMatrixLookAtLH(XMVectorSet(0.0f, 0.0f, -6.0f, 1.0f),
            XMVectorZero(), XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
        const XMMATRIX projection = XMMatrixPerspectiveFovLH(XM_PIDIV4,
            static_cast<float>(width_) / static_cast<float>(height_), 0.1f, 100.0f);
        draw_constants draw{};
        XMStoreFloat4x4(&draw.view_projection, view * projection);
        XMStoreFloat4x4(&draw.world, XMMatrixIdentity());

        const DirectX::XMFLOAT4 colors[2] = {
            DirectX::XMFLOAT4(0.12f, 0.58f, 0.96f, 1.0f),
            DirectX::XMFLOAT4(0.98f, 0.36f, 0.13f, 1.0f)
        };
        for (std::uint32_t body_index = 0; body_index < 2; ++body_index)
        {
            const D3D12_VERTEX_BUFFER_VIEW& vertex_view = use_camera_surface
                ? compute_->camera_bodies[body_index].vertex_view
                : compute_->bodies[body_index].vertex_view;
            command_list_->IASetVertexBuffers(0, 1, &vertex_view);
            draw.color = colors[body_index];
            command_list_->SetGraphicsRoot32BitConstants(0, 36, &draw, 0);
            command_list_->DrawIndexedInstanced(index_count_, 1, 0, 0, 0);
        }

        command_list_->EndQuery(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 4);
        command_list_->ResolveQueryData(timestamp_query_heap_.Get(), D3D12_QUERY_TYPE_TIMESTAMP,
            0, 5u + max_solver_iterations * 8u, timestamp_readback_.Get(), 0);

        back_buffer_barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        back_buffer_barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        command_list_->ResourceBarrier(1, &back_buffer_barrier);
        if (FAILED(command_list_->Close())) throw std::runtime_error("Command list close failed.");
        ID3D12CommandList* command_lists[] = {command_list_.Get()};
        command_queue_->ExecuteCommandLists(1, command_lists);
        if (FAILED(swap_chain_->Present(1, 0))) throw std::runtime_error("Swap-chain present failed.");

        const std::uint64_t signal_value = next_fence_value_++;
        if (FAILED(command_queue_->Signal(fence_.Get(), signal_value)))
            throw std::runtime_error("Queue signal failed.");
        frame_fence_values_[frame_index_] = signal_value;
        if (fence_->GetCompletedValue() < signal_value)
        {
            if (FAILED(fence_->SetEventOnCompletion(signal_value, fence_event_)))
                throw std::runtime_error("Fence wait setup failed.");
            WaitForSingleObject(fence_event_, INFINITE);
        }
        frame_index_ = swap_chain_->GetCurrentBackBufferIndex();

        const std::chrono::steady_clock::time_point cpu_end = std::chrono::steady_clock::now();
        const double cpu_frame_ms = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();
        resolve_benchmark_queries(cpu_frame_ms);
    }

    void d3d12_renderer::set_solver_mode(solver_mode mode)
    {
        solver_mode_ = mode;
    }

    solver_mode d3d12_renderer::get_solver_mode() const
    {
        return solver_mode_;
    }

    void d3d12_renderer::set_simulation_parameters(const simulation_parameters& parameters)
    {
        simulation_parameters_.solver_iterations = std::clamp(parameters.solver_iterations, 1u, max_solver_iterations);
        simulation_parameters_.edge_relaxation = std::clamp(parameters.edge_relaxation, 0.0f, 1.5f);
        simulation_parameters_.edge_compliance = std::clamp(parameters.edge_compliance, 1.0e-9f, 1.0e-2f);
        simulation_parameters_.area_compliance = std::clamp(parameters.area_compliance, 1.0e-9f, 1.0e-2f);
        simulation_parameters_.bending_compliance = std::clamp(parameters.bending_compliance, 1.0e-9f, 1.0e-2f);
        simulation_parameters_.shape_matching_strength = std::clamp(parameters.shape_matching_strength, 0.0f, 1.0f);
        simulation_parameters_.bone_follow_strength = std::clamp(parameters.bone_follow_strength, 0.0f, 1.0f);
        simulation_parameters_.max_correction = std::clamp(parameters.max_correction, 0.0001f, 0.05f);
        simulation_parameters_.particle_radius = std::clamp(parameters.particle_radius, 0.001f, 0.05f);
        simulation_parameters_.pushout = std::clamp(parameters.pushout, 0.0f, 0.03f);
        simulation_parameters_.velocity_friction = std::clamp(parameters.velocity_friction, 0.0f, 1.0f);
        simulation_parameters_.restitution = std::clamp(parameters.restitution, 0.0f, 1.0f);
        simulation_parameters_.contact_band = std::clamp(parameters.contact_band, 0.005f, 0.5f);
    }

    simulation_parameters d3d12_renderer::get_simulation_parameters() const
    {
        return simulation_parameters_;
    }

    performance_metrics d3d12_renderer::get_performance_metrics() const
    {
        return performance_metrics_;
    }

    void d3d12_renderer::set_simulation_paused(bool paused)
    {
        simulation_paused_ = paused;
    }

    bool d3d12_renderer::is_simulation_paused() const
    {
        return simulation_paused_;
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
