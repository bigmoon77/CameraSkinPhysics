struct Particle { float4 position; float4 velocity; };
Texture2D<float> g_scene_depth : register(t0);
Texture2D<float4> g_scene_normal : register(t1);
StructuredBuffer<Particle> g_particles_in : register(t2);
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

void sample_contact_surface(uint2 pixel, out float scene_depth, out float3 scene_normal)
{
    scene_depth = g_scene_depth.Load(int3(pixel, 0));
    scene_normal = decode_scene_normal(pixel);
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
    float scene_depth;
    float3 normal;
    sample_contact_surface(pixel, scene_depth, normal);
    if (scene_depth >= 1.0f)
        return 0.0f;

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
    {
        g_contact_corrections[index] = float4(raw_correction, has_contact ? 1.0f : 0.0f);
        particle.position.xyz += raw_correction;
    }
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
