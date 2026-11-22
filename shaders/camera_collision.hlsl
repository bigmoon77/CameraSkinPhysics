struct Particle { float4 position; float4 velocity; };
Texture2D<float> g_scene_depth : register(t0);
Texture2D<float4> g_scene_normal : register(t1);
StructuredBuffer<Particle> g_particles_in : register(t2);
RWStructuredBuffer<Particle> g_particles_out : register(u0);
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
};
[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;
    Particle particle = g_particles_in[index];
    const float4 clip = mul(float4(particle.position.xyz, 1.0f), view_projection);
    if (clip.w <= 0.0f) { g_particles_out[index] = particle; return; }
    const float3 ndc = clip.xyz / clip.w;
    const float2 uv = float2(ndc.x * 0.5f + 0.5f, 0.5f - ndc.y * 0.5f);
    if (any(uv < 0.0f) || any(uv >= 1.0f) || ndc.z < 0.0f || ndc.z > 1.0f)
    { g_particles_out[index] = particle; return; }
    const uint2 pixel = min(uint2(uv * viewport_size), viewport_size - 1);
    const float scene_depth = g_scene_depth.Load(int3(pixel, 0));
    if (scene_depth >= 1.0f) { g_particles_out[index] = particle; return; }
    const float3 normal = normalize(g_scene_normal.Load(int3(pixel, 0)).xyz * 2.0f - 1.0f);
    const float4 surface_clip = float4(ndc.xy, scene_depth, 1.0f);
    const float4 surface_world_h = mul(surface_clip, inverse_view_projection);
    const float3 surface_world = surface_world_h.xyz / max(abs(surface_world_h.w), 1.0e-8f);
    const float signed_distance = dot(particle.position.xyz - surface_world, normal);

    // A single frontmost depth layer cannot classify points deep behind the
    // captured surface. Limit the correction to a thin contact band to avoid
    // projecting every interior particle onto the camera-facing shell.
    const float effective_contact_band = max(contact_band, particle_radius * 4.0f + pushout);
    if (signed_distance < -effective_contact_band)
    {
        g_particles_out[index] = particle;
        return;
    }

    const float penetration = particle_radius + pushout - signed_distance;
    if (penetration > 0.0f)
    {
        particle.position.xyz += normal * min(penetration, effective_contact_band);
        const float normal_velocity = dot(particle.velocity.xyz, normal);
        if (normal_velocity < 0.0f)
        {
            const float3 tangent_velocity = particle.velocity.xyz - normal * normal_velocity;
            particle.velocity.xyz = tangent_velocity * (1.0f - velocity_friction)
                - normal * normal_velocity * restitution;
        }
    }
    g_particles_out[index] = particle;
}
