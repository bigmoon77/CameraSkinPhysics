struct Particle { float4 position; float4 velocity; };
struct Tetrahedron { uint4 indices; float rest_volume; float compliance; float lambda; float padding; };
StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<Tetrahedron> g_tetrahedra : register(t1);
RWStructuredBuffer<float4> g_tet_corrections : register(u0);
RWStructuredBuffer<float> g_tet_lambdas : register(u1);
cbuffer TetrahedronConstants : register(b0)
{
    uint tetrahedron_count;
    float delta_time;
    float relaxation;
    float padding;
};
float signed_volume(float3 a, float3 b, float3 c, float3 d)
{
    return dot(cross(b - a, c - a), d - a) / 6.0f;
}
[numthreads(128, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= tetrahedron_count) return;
    const Tetrahedron tet = g_tetrahedra[index];
    const float3 p0 = g_particles[tet.indices.x].position.xyz;
    const float3 p1 = g_particles[tet.indices.y].position.xyz;
    const float3 p2 = g_particles[tet.indices.z].position.xyz;
    const float3 p3 = g_particles[tet.indices.w].position.xyz;
    const float constraint = signed_volume(p0, p1, p2, p3) - tet.rest_volume;
    const float3 g1 = cross(p2 - p0, p3 - p0) / 6.0f;
    const float3 g2 = cross(p3 - p0, p1 - p0) / 6.0f;
    const float3 g3 = cross(p1 - p0, p2 - p0) / 6.0f;
    const float3 g0 = -(g1 + g2 + g3);
    const float alpha = tet.compliance / max(delta_time * delta_time, 1.0e-8f);
    const float denominator = dot(g0,g0) + dot(g1,g1) + dot(g2,g2) + dot(g3,g3) + alpha;
    const float delta_lambda = (-constraint - alpha * tet.lambda) / max(denominator, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;
    g_tet_lambdas[index] = tet.lambda + scaled_lambda;
    g_tet_corrections[index] = float4(g0 * scaled_lambda, 0.0f);
}
