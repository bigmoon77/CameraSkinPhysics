struct Particle
{
    float4 position;
    float4 velocity;
};

StructuredBuffer<float4> g_corrections : register(t0);
StructuredBuffer<uint2> g_edge_ranges : register(t1);
StructuredBuffer<uint> g_incident_neighbors : register(t2);
StructuredBuffer<Particle> g_particles_in : register(t4);
RWStructuredBuffer<Particle> g_particles_out : register(u1);

cbuffer ApplyConstants : register(b0)
{
    uint particle_count;
    float blend_strength;
    uint padding0;
    uint padding1;
};

[numthreads(256, 1, 1)]
void apply_corrections(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;

    const float4 own = g_corrections[index];
    const float own_length = length(own.xyz);
    const float3 own_direction = own_length > 1.0e-7f ? own.xyz / own_length : 0.0f;
    float3 sum = own.xyz;
    float weight_sum = 1.0f;
    const uint2 range = g_edge_ranges[index];
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint neighbor_index = g_incident_neighbors[range.x + offset];
        if (neighbor_index >= particle_count || neighbor_index == index)
            continue;

        const float4 neighbor = g_corrections[neighbor_index];
        const float neighbor_length = length(neighbor.xyz);
        float weight = 1.0f;
        if (own_length > 1.0e-7f && neighbor_length > 1.0e-7f)
        {
            const float direction_similarity = dot(own_direction, neighbor.xyz / neighbor_length);
            if (direction_similarity < 0.25f)
                continue;
            weight = saturate((direction_similarity - 0.25f) / 0.75f);
        }

        sum += neighbor.xyz * weight;
        weight_sum += weight;
    }

    const float3 average = sum / weight_sum;
    const float3 smoothed = lerp(own.xyz, average, saturate(blend_strength));
    Particle particle = g_particles_in[index];
    particle.position.xyz += smoothed - own.xyz;
    g_particles_out[index] = particle;
}
