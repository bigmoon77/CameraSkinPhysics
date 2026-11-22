struct Particle
{
    float4 position;
    float4 velocity;
};

StructuredBuffer<Particle> g_particles_in : register(t0);
StructuredBuffer<uint2> g_gather_ranges : register(t1);
StructuredBuffer<uint> g_gather_incidents : register(t2);
StructuredBuffer<float3> g_surface_corrections : register(t3);
RWStructuredBuffer<Particle> g_particles_out : register(u0);

cbuffer GatherConstants : register(b0)
{
    uint particle_count;
    float max_correction;
    float delta_time;
    uint edge_count;
    uint triangle_count;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint particle_index = dispatch_id.x;
    if (particle_index >= particle_count) return;

    Particle particle = g_particles_in[particle_index];
    const uint2 range = g_gather_ranges[particle_index];
    float3 correction = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint encoded = g_gather_incidents[range.x + offset];
        const uint correction_index = encoded & 0x7fffffffu;
        const float sign = (encoded & 0x80000000u) != 0u ? -1.0f : 1.0f;
        correction += g_surface_corrections[correction_index] * sign;
    }

    const float correction_length_squared = dot(correction, correction);
    const float max_correction_squared = max_correction * max_correction;
    if (correction_length_squared > max_correction_squared)
        correction *= max_correction * rsqrt(correction_length_squared);

    particle.position.xyz += correction;
    particle.velocity.xyz += correction / max(delta_time, 1.0e-6f);
    g_particles_out[particle_index] = particle;
}
