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
RWStructuredBuffer<SkinVertex> g_vertices : register(u0);

cbuffer SkinConstants : register(b0)
{
    uint vertex_count;
    float3 local_center;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= vertex_count) return;

    const float3 position = g_particles[index].position.xyz;
    const float3 radial = position - local_center;
    SkinVertex output;
    output.position = position;
    output.normal = dot(radial, radial) > 1.0e-12f
        ? normalize(radial)
        : float3(0.0f, 1.0f, 0.0f);
    g_vertices[index] = output;
}
