#pragma once
#include "sphere_mesh.hpp"
#include "skinned_animation.hpp"
#include "tetrahedral_mesh.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <wrl.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <windows.h>

namespace csp
{
    inline constexpr std::uint32_t max_solver_iterations = 16;

    enum class solver_mode
    {
        tetrahedral_xpbd,
        camera_surface
    };

    struct simulation_parameters
    {
        std::uint32_t solver_iterations = 4;
        float edge_relaxation = 0.8f;
        float edge_compliance = 2.0e-6f;
        float area_compliance = 1.0e-7f;
        float bending_compliance = 2.0e-4f;
        float shape_matching_strength = 0.25f;
        float bone_follow_strength = 0.35f;
        float max_correction = 0.008f;
        float particle_radius = 0.012f;
        float pushout = 0.006f;
        float velocity_friction = 0.12f;
        float restitution = 0.18f;
        float contact_band = 0.10f;
    };

    struct performance_metrics
    {
        double cpu_frame_ms = 0.0;
        double gpu_simulation_ms = 0.0;
        double camera_constraint_solve_ms = 0.0;
        double camera_edge_solve_ms = 0.0;
        double camera_area_solve_ms = 0.0;
        double camera_bending_solve_ms = 0.0;
        double camera_gather_ms = 0.0;
        double gpu_depth_capture_ms = 0.0;
        double gpu_collision_ms = 0.0;
        double gpu_render_ms = 0.0;
        double gpu_total_ms = 0.0;
    };

    class d3d12_renderer
    {
    public:
        d3d12_renderer();
        ~d3d12_renderer();
        d3d12_renderer(const d3d12_renderer&) = delete;
        d3d12_renderer& operator=(const d3d12_renderer&) = delete;

        bool initialize(HWND window, std::uint32_t width, std::uint32_t height,
            const sphere_mesh& surface_mesh, const tetrahedral_mesh& volume_mesh,
            const skeletal_animation* animation = nullptr);
        void render(float delta_time);
        void set_solver_mode(solver_mode mode);
        solver_mode get_solver_mode() const;
        void set_simulation_parameters(const simulation_parameters& parameters);
        simulation_parameters get_simulation_parameters() const;
        performance_metrics get_performance_metrics() const;
        void set_simulation_paused(bool paused);
        bool is_simulation_paused() const;
        void wait_gpu();

    private:
        struct compute_state;
        static constexpr std::uint32_t frame_count = 2;
        static constexpr std::uint32_t capture_count = 2;
        static constexpr std::uint32_t capture_size = 1024;

        void create_render_targets();
        void create_capture_targets();
        bool create_sphere_buffers(const sphere_mesh& mesh);
        bool create_depth_buffer();
        bool create_sphere_pipeline();
        bool create_capture_pipeline();
        bool create_compute_resources(const sphere_mesh& surface_mesh, const tetrahedral_mesh& volume_mesh,
            const skeletal_animation* animation);
        bool create_compute_pipelines();

        std::uint32_t reserve_descriptor_table();
        void clear_descriptor_table(std::uint32_t base);
        void write_buffer_srv(std::uint32_t base, std::uint32_t slot,
            ID3D12Resource* resource, std::uint32_t element_count, std::uint32_t stride);
        void write_texture_srv(std::uint32_t base, std::uint32_t slot,
            ID3D12Resource* resource, DXGI_FORMAT format);
        void write_buffer_uav(std::uint32_t base, std::uint32_t slot,
            ID3D12Resource* resource, std::uint32_t element_count, std::uint32_t stride);
        void dispatch_compute(ID3D12PipelineState* pipeline, std::uint32_t base,
            const void* constants, std::uint32_t constant_count, std::uint32_t group_count);
        void transition_resource(ID3D12Resource* resource,
            D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after);
        void simulate_volume(float delta_time);
        void simulate_camera_surface(float delta_time);
        void update_skeletal_pose(float delta_time);
        void skin_body(std::uint32_t body_index);
        void skin_camera_body(std::uint32_t body_index);
        void capture_body(std::uint32_t capture_index, std::uint32_t body_index);
        void resolve_benchmark_queries(double cpu_frame_ms);

        std::unique_ptr<compute_state> compute_;

        Microsoft::WRL::ComPtr<IDXGIFactory6> factory_;
        Microsoft::WRL::ComPtr<ID3D12Device> device_;
        Microsoft::WRL::ComPtr<ID3D12CommandQueue> command_queue_;
        Microsoft::WRL::ComPtr<IDXGISwapChain3> swap_chain_;
        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtv_heap_;
        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> dsv_heap_;
        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> capture_rtv_heap_;
        Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> capture_dsv_heap_;
        Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> command_list_;
        Microsoft::WRL::ComPtr<ID3D12Fence> fence_;
        Microsoft::WRL::ComPtr<ID3D12RootSignature> graphics_root_signature_;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> graphics_pipeline_;
        Microsoft::WRL::ComPtr<ID3D12RootSignature> capture_root_signature_;
        Microsoft::WRL::ComPtr<ID3D12PipelineState> capture_pipeline_;
        Microsoft::WRL::ComPtr<ID3D12Resource> depth_buffer_;
        Microsoft::WRL::ComPtr<ID3D12Resource> index_buffer_;
        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, capture_count> capture_normal_;
        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, capture_count> capture_depth_;
        Microsoft::WRL::ComPtr<ID3D12QueryHeap> timestamp_query_heap_;
        Microsoft::WRL::ComPtr<ID3D12Resource> timestamp_readback_;
        std::array<Microsoft::WRL::ComPtr<ID3D12CommandAllocator>, frame_count> command_allocators_;
        std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, frame_count> render_targets_;
        std::array<std::uint64_t, frame_count> frame_fence_values_{};
        D3D12_INDEX_BUFFER_VIEW index_buffer_view_{};
        std::uint32_t index_count_ = 0;
        std::uint32_t width_ = 0;
        std::uint32_t height_ = 0;
        std::uint32_t rtv_descriptor_size_ = 0;
        std::uint32_t dsv_descriptor_size_ = 0;
        std::uint32_t capture_rtv_descriptor_size_ = 0;
        std::uint32_t capture_dsv_descriptor_size_ = 0;
        std::uint32_t frame_index_ = 0;
        std::uint64_t next_fence_value_ = 1;
        std::uint64_t timestamp_frequency_ = 0;
        HANDLE fence_event_ = nullptr;
        solver_mode solver_mode_ = solver_mode::tetrahedral_xpbd;
        simulation_parameters simulation_parameters_{};
        performance_metrics performance_metrics_{};
        bool simulation_paused_ = false;
    };
}
