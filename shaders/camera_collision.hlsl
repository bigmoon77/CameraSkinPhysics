struct Particle { float4 position; float4 velocity; };
struct SurfaceEdge
{
    uint2 indices;
    float rest_length;
    float compliance;
};

Texture2D<float> g_scene_depth : register(t0);
Texture2D<float4> g_scene_normal : register(t1);
StructuredBuffer<Particle> g_particles_in : register(t2);
StructuredBuffer<uint2> g_edge_ranges : register(t3);
StructuredBuffer<uint> g_incident_edge_corrections : register(t4);
StructuredBuffer<SurfaceEdge> g_surface_edges : register(t5);
RWStructuredBuffer<Particle> g_particles_out : register(u0);
RWStructuredBuffer<float4> g_contact_corrections : register(u1);

cbuffer CameraCollisionConstants : register(b0)
{
    row_major float4x4 view_projection;
    row_major float4x4 inverse_view_projection;
    uint particle_count;
    uint2 viewport_size;
    float particle_radius;
    float pushout;
    float velocity_friction;
    float restitution;
    float contact_band;
    uint smooth_surface_contacts;
};

float3 decode_scene_normal(uint2 pixel)
{
    return normalize(g_scene_normal.Load(int3(pixel, 0)).xyz * 2.0f - 1.0f);
}

void sample_filtered_contact(uint2 center_pixel, out float filtered_depth, out float3 filtered_normal)
{
    const float center_depth = g_scene_depth.Load(int3(center_pixel, 0));
    const float3 center_normal = decode_scene_normal(center_pixel);
    const float depth_threshold = max(0.025f, particle_radius * 2.0f + pushout * 2.0f);
    const float normal_threshold = 0.5f;
    const float depth_sigma = max(depth_threshold * 0.5f, 0.01f);
    float depth_sum = 0.0f;
    float3 normal_sum = 0.0f;
    float weight_sum = 0.0f;

    [unroll]
    for (int y = -1; y <= 1; ++y)
    {
        [unroll]
        for (int x = -1; x <= 1; ++x)
        {
            const int2 sample_position = int2(center_pixel) + int2(x, y);
            if (any(sample_position < 0) || any(sample_position >= int2(viewport_size)))
                continue;

            const uint2 sample_pixel = uint2(sample_position);
            const float sample_depth = g_scene_depth.Load(int3(sample_pixel, 0));
            if (sample_depth >= 1.0f)
                continue;

            const float depth_difference = abs(sample_depth - center_depth) * 19.9f;
            if (depth_difference > depth_threshold)
                continue;

            const float3 sample_normal = decode_scene_normal(sample_pixel);
            const float normal_similarity = dot(center_normal, sample_normal);
            if (normal_similarity < normal_threshold)
                continue;

            const float spatial_weight = (x == 0 ? 2.0f : 1.0f) * (y == 0 ? 2.0f : 1.0f);
            const float depth_weight = exp(-depth_difference / depth_sigma);
            const float normal_weight = saturate((normal_similarity - normal_threshold)
                / (1.0f - normal_threshold));
            const float weight = spatial_weight * depth_weight * normal_weight;
            depth_sum += sample_depth * weight;
            normal_sum += sample_normal * weight;
            weight_sum += weight;
        }
    }

    if (weight_sum <= 1.0e-6f)
    {
        filtered_depth = center_depth;
        filtered_normal = center_normal;
        return;
    }

    filtered_depth = depth_sum / weight_sum;
    filtered_normal = normalize(normal_sum / weight_sum);
}

float3 evaluate_contact_correction(Particle particle,
    out float3 contact_normal, out bool has_contact)
{
    contact_normal = float3(0.0f, 1.0f, 0.0f);
    has_contact = false;

    const float4 clip = mul(float4(particle.position.xyz, 1.0f), view_projection);
    if (clip.w <= 0.0f)
        return 0.0f;

    const float3 ndc = clip.xyz / clip.w;
    const float2 uv = float2(ndc.x * 0.5f + 0.5f, 0.5f - ndc.y * 0.5f);
    if (any(uv < 0.0f) || any(uv >= 1.0f) || ndc.z < 0.0f || ndc.z > 1.0f)
        return 0.0f;

    const uint2 pixel = min(uint2(uv * viewport_size), viewport_size - 1);
    const float center_depth = g_scene_depth.Load(int3(pixel, 0));
    if (center_depth >= 1.0f)
        return 0.0f;

    float scene_depth = center_depth;
    float3 normal = decode_scene_normal(pixel);
    sample_filtered_contact(pixel, scene_depth, normal);

    const float4 surface_clip = float4(ndc.xy, scene_depth, 1.0f);
    const float4 surface_world_h = mul(surface_clip, inverse_view_projection);
    const float3 surface_world = surface_world_h.xyz / max(abs(surface_world_h.w), 1.0e-8f);
    const float signed_distance = dot(particle.position.xyz - surface_world, normal);
    const float effective_contact_band = max(contact_band, particle_radius * 4.0f + pushout);
    if (signed_distance < -effective_contact_band)
        return 0.0f;

    has_contact = true;
    contact_normal = normal;
    const float penetration = particle_radius + pushout - signed_distance;
    if (penetration <= 0.0f)
        return 0.0f;

    return normal * min(penetration, effective_contact_band);
}

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;
    Particle particle = g_particles_in[index];
    float3 contact_normal;
    bool has_contact;
    const float3 raw_correction = evaluate_contact_correction(particle, contact_normal, has_contact);
    if (smooth_surface_contacts != 0u)
        g_contact_corrections[index] = float4(raw_correction, has_contact ? 1.0f : 0.0f);
    if (has_contact)
    {
        const float normal_velocity = dot(particle.velocity.xyz, contact_normal);
        if (normal_velocity < 0.0f)
        {
            const float3 tangent_velocity = particle.velocity.xyz - contact_normal * normal_velocity;
            particle.velocity.xyz = tangent_velocity * (1.0f - velocity_friction)
                - contact_normal * normal_velocity * restitution;
        }
    }
    g_particles_out[index] = particle;
}
