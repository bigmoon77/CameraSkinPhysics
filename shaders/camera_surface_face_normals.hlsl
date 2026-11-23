struct Particle
{
    float4 position;
    float4 velocity;
};

struct TriangleArea
{
    uint3 indices;
    float rest_area;
    float compliance;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<TriangleArea> g_triangles : register(t1);
RWStructuredBuffer<float4> g_face_normals : register(u0);

cbuffer FaceNormalConstants : register(b0)
{
    uint triangle_count;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint triangle_index = dispatch_id.x;
    if (triangle_index >= triangle_count) return;

    const TriangleArea triangle = g_triangles[triangle_index];
    const float3 a = g_particles[triangle.indices.x].position.xyz;
    const float3 b = g_particles[triangle.indices.y].position.xyz;
    const float3 c = g_particles[triangle.indices.z].position.xyz;
    const float3 face_normal = cross(b - a, c - a);
    g_face_normals[triangle_index] = float4(face_normal, 0.0f);
}
