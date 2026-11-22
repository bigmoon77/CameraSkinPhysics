struct Particle
{
    float4 position;
    float4 velocity;
};

StructuredBuffer<Particle> g_particles_in : register(t0);
StructuredBuffer<uint2> g_incidence_ranges : register(t1);
StructuredBuffer<uint> g_incident_corrections : register(t2);
StructuredBuffer<float3> g_surface_corrections : register(t3);
StructuredBuffer<uint2> g_triangle_incidence_ranges : register(t4);
StructuredBuffer<uint> g_incident_triangle_corners : register(t5);
StructuredBuffer<uint2> g_bending_incidence_ranges : register(t6);
StructuredBuffer<uint> g_incident_bending_corners : register(t7);
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
    const uint2 edge_range = g_incidence_ranges[particle_index];
    float3 correction = 0.0f;
    for (uint offset = 0; offset < edge_range.y; ++offset)
    {
        const uint encoded = g_incident_corrections[edge_range.x + offset];
        const uint edge_index = encoded >> 1u;
        const float endpoint_sign = (encoded & 1u) != 0u ? -1.0f : 1.0f;
        correction += g_surface_corrections[edge_index] * endpoint_sign;
    }

    const uint2 triangle_range = g_triangle_incidence_ranges[particle_index];
    for (uint offset = 0; offset < triangle_range.y; ++offset)
    {
        const uint triangle_corner = g_incident_triangle_corners[triangle_range.x + offset];
        correction += g_surface_corrections[edge_count + triangle_corner];
    }

    const uint2 bending_range = g_bending_incidence_ranges[particle_index];
    for (uint offset = 0; offset < bending_range.y; ++offset)
    {
        const uint hinge_corner = g_incident_bending_corners[bending_range.x + offset];
        const uint hinge_index = hinge_corner >> 1u;
        const uint opposite_index = hinge_corner & 1u;
        const uint correction_index = edge_count + triangle_count * 3u + hinge_index * 2u + opposite_index;
        correction += g_surface_corrections[correction_index];
    }

    const float correction_length = length(correction);
    if (correction_length > max_correction)
        correction *= max_correction / correction_length;

    particle.position.xyz += correction;
    particle.velocity.xyz += correction / max(delta_time, 1.0e-6f);
    g_particles_out[particle_index] = particle;
}
