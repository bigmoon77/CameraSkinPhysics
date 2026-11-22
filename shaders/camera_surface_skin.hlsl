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

struct TriangleArea
{
    uint3 indices;
    float rest_area;
    float compliance;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<uint2> g_vertex_triangle_ranges : register(t1);
StructuredBuffer<uint> g_incident_triangle_corners : register(t2);
StructuredBuffer<TriangleArea> g_triangles : register(t3);
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

    const float3 position = g_particles[vertex_index].position.xyz;
    const uint2 range = g_vertex_triangle_ranges[vertex_index];
    float3 normal_sum = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint encoded_corner = g_incident_triangle_corners[range.x + offset];
        const uint triangle_index = encoded_corner / 3u;
        const TriangleArea triangle = g_triangles[triangle_index];
        const float3 a = g_particles[triangle.indices.x].position.xyz;
        const float3 b = g_particles[triangle.indices.y].position.xyz;
        const float3 c = g_particles[triangle.indices.z].position.xyz;
        normal_sum += cross(b - a, c - a);
    }

    SkinVertex output;
    output.position = position;
    output.normal = dot(normal_sum, normal_sum) > 1.0e-12f
        ? normalize(normal_sum)
        : float3(0.0f, 1.0f, 0.0f);
    g_vertices[vertex_index] = output;
}
