struct Particle
{
    float4 position;
    float4 velocity;
};

struct SurfaceMapping
{
    uint4 indices;
    float4 weights;
    float4 rest_offset;
};

struct SkinVertex
{
    float3 position;
    float3 normal;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<SurfaceMapping> g_surface_mappings : register(t1);
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

    const SurfaceMapping mapping = g_surface_mappings[index];
    const float3 p0 = g_particles[mapping.indices.x].position.xyz;
    const float3 p1 = g_particles[mapping.indices.y].position.xyz;
    const float3 p2 = g_particles[mapping.indices.z].position.xyz;
    const float3 p3 = g_particles[mapping.indices.w].position.xyz;

    SkinVertex output;
    output.position = p0 * mapping.weights.x
        + p1 * mapping.weights.y
        + p2 * mapping.weights.z
        + p3 * mapping.weights.w
        + mapping.rest_offset.xyz;

    const float3 radial = output.position - local_center;
    output.normal = dot(radial, radial) > 1.0e-12f ? normalize(radial) : float3(0.0f, 1.0f, 0.0f);
    g_vertices[index] = output;
}
