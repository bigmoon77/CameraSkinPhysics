struct Particle
{
    float4 position;
    float4 velocity;
};

struct SkinVertex
{
    float3 position;
    float3 normal;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<uint2> g_vertex_triangle_ranges : register(t1);
StructuredBuffer<uint> g_incident_triangle_corners : register(t2);
StructuredBuffer<float4> g_face_normals : register(t3);
RWStructuredBuffer<SkinVertex> g_vertices : register(u0);

cbuffer SkinConstants : register(b0)
{
    uint vertex_count;
    float3 local_center;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_id.x;
    if (vertex_index >= vertex_count) return;

    const uint2 range = g_vertex_triangle_ranges[vertex_index];
    float3 normal_sum = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint encoded_corner = g_incident_triangle_corners[range.x + offset];
        const uint triangle_index = encoded_corner / 3u;
        normal_sum += g_face_normals[triangle_index].xyz;
    }

    SkinVertex output;
    output.position = g_particles[vertex_index].position.xyz;
    output.normal = dot(normal_sum, normal_sum) > 1.0e-12f
        ? normalize(normal_sum)
        : float3(0.0f, 1.0f, 0.0f);
    g_vertices[vertex_index] = output;
}
