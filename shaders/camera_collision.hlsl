struct Particle { float4 position; float4 velocity; };
Texture2D<float> g_scene_depth : register(t0);
Texture2D<float4> g_scene_normal : register(t1);
StructuredBuffer<Particle> g_particles_in : register(t2);
RWStructuredBuffer<Particle> g_particles_out : register(u0);
SamplerState g_point_clamp : register(s0);
cbuffer CameraCollisionConstants : register(b0)
{
    row_major float4x4 view_projection;
    uint particle_count;
    float particle_radius;
    float pushout;
    float depth_bias;
    float2 inverse_viewport;
    float2 padding;
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
    if (any(uv < 0.0f) || any(uv > 1.0f) || ndc.z < 0.0f || ndc.z > 1.0f)
    { g_particles_out[index] = particle; return; }
    const float scene_depth = g_scene_depth.SampleLevel(g_point_clamp, uv, 0);
    if (scene_depth >= 1.0f) { g_particles_out[index] = particle; return; }
    const float3 normal = normalize(g_scene_normal.SampleLevel(g_point_clamp, uv, 0).xyz * 2.0f - 1.0f);
    if (scene_depth - ndc.z < -depth_bias)
    {
        particle.position.xyz += normal * (particle_radius + pushout);
        const float inward_velocity = dot(particle.velocity.xyz, normal);
        if (inward_velocity < 0.0f) particle.velocity.xyz -= normal * inward_velocity;
    }
    g_particles_out[index] = particle;
}
