struct Particle
{
    float4 position;
    float4 velocity;
};

StructuredBuffer<Particle> g_particles_in : register(t0);
StructuredBuffer<uint2> g_incidence_ranges : register(t1);
StructuredBuffer<uint> g_incident_corrections : register(t2);
StructuredBuffer<float3> g_edge_corrections : register(t3);
StructuredBuffer<uint> g_gather_vertex_indices : register(t4);
RWStructuredBuffer<Particle> g_particles_out : register(u0);

cbuffer GatherConstants : register(b0)
{
    uint particle_count;
    float max_correction;
    float delta_time;
    float padding;
};

static const uint low_degree_limit = 8u;

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint dispatch_index = dispatch_id.x;
    if (dispatch_index >= particle_count) return;

    // The CPU builds this list in ascending degree order. Most GPU waves therefore
    // contain only low-degree vertices, leaving the variable-length path to the tail.
    const uint particle_index = g_gather_vertex_indices[dispatch_index];
    Particle particle = g_particles_in[particle_index];
    const uint2 range = g_incidence_ranges[particle_index];
    float3 correction = 0.0f;

    if (range.y <= low_degree_limit)
    {
        [unroll]
        for (uint offset = 0; offset < low_degree_limit; ++offset)
        {
            if (offset >= range.y) break;
            const uint correction_index = g_incident_corrections[range.x + offset];
            const uint edge_index = correction_index >> 1u;
            const float endpoint_sign = (correction_index & 1u) != 0u ? -1.0f : 1.0f;
            correction += g_edge_corrections[edge_index] * endpoint_sign;
        }
    }
    else
    {
        for (uint offset = 0; offset < range.y; ++offset)
        {
            const uint correction_index = g_incident_corrections[range.x + offset];
            const uint edge_index = correction_index >> 1u;
            const float endpoint_sign = (correction_index & 1u) != 0u ? -1.0f : 1.0f;
            correction += g_edge_corrections[edge_index] * endpoint_sign;
        }
    }

    const float correction_length = length(correction);
    if (correction_length > max_correction)
        correction *= max_correction / correction_length;

    particle.position.xyz += correction;
    particle.velocity.xyz += correction / max(delta_time, 1.0e-6f);
    g_particles_out[particle_index] = particle;
}
