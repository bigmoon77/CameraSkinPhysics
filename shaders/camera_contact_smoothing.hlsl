struct Particle
{
    float4 position;
    float4 velocity;
};

struct SurfaceEdge
{
    uint2 indices;
    float rest_length;
    float compliance;
};

StructuredBuffer<float4> g_corrections_in : register(t0);
StructuredBuffer<uint2> g_edge_ranges : register(t1);
StructuredBuffer<uint> g_incident_edge_corrections : register(t2);
StructuredBuffer<SurfaceEdge> g_surface_edges : register(t3);
RWStructuredBuffer<float4> g_corrections_out : register(u0);

cbuffer SmoothConstants : register(b0)
{
    uint particle_count;
    float blend_strength;
    uint padding0;
    uint padding1;
};

[numthreads(256, 1, 1)]
void smooth_corrections(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;
    const float4 own = g_corrections_in[index];
    if (own.w <= 0.0f)
    {
        g_corrections_out[index] = own;
        return;
    }

    float3 sum = own.xyz;
    float weight_sum = 1.0f;
    const uint2 range = g_edge_ranges[index];
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint encoded_edge = g_incident_edge_corrections[range.x + offset];
        const SurfaceEdge edge = g_surface_edges[encoded_edge >> 1u];
        const uint neighbor_index = (encoded_edge & 1u) != 0u
            ? edge.indices.x : edge.indices.y;
        if (neighbor_index >= particle_count || neighbor_index == index)
            continue;

        const float4 neighbor = g_corrections_in[neighbor_index];
        if (neighbor.w <= 0.0f)
            continue;

        const float own_length = length(own.xyz);
        const float neighbor_length = length(neighbor.xyz);
        if (own_length <= 1.0e-7f || neighbor_length <= 1.0e-7f)
            continue;

        const float direction_similarity = dot(normalize(own.xyz), normalize(neighbor.xyz));
        if (direction_similarity < 0.25f)
            continue;

        const float weight = saturate((direction_similarity - 0.25f) / 0.75f);
        sum += neighbor.xyz * weight;
        weight_sum += weight;
    }

    const float3 average = sum / weight_sum;
    g_corrections_out[index] = float4(
        lerp(own.xyz, average, saturate(blend_strength)), own.w);
}

StructuredBuffer<Particle> g_particles_in : register(t4);
StructuredBuffer<float4> g_final_corrections : register(t5);
RWStructuredBuffer<Particle> g_particles_out : register(u1);

[numthreads(256, 1, 1)]
void apply_corrections(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;
    Particle particle = g_particles_in[index];
    particle.position.xyz += g_final_corrections[index].xyz;
    g_particles_out[index] = particle;
}
