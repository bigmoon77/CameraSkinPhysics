struct Particle { float4 position; float4 velocity; };
struct Tetrahedron { uint4 indices; float rest_volume; float compliance; float lambda; float padding; };
StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<Tetrahedron> g_tetrahedra : register(t1);
StructuredBuffer<float> g_inverse_masses : register(t2);
StructuredBuffer<float> g_tet_lambda_in : register(t3);
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
    const uint4 ids = tet.indices;
    const float3 p0 = g_particles[ids.x].position.xyz;
    const float3 p1 = g_particles[ids.y].position.xyz;
    const float3 p2 = g_particles[ids.z].position.xyz;
    const float3 p3 = g_particles[ids.w].position.xyz;
    const float constraint = signed_volume(p0, p1, p2, p3) - tet.rest_volume;
    const float3 g1 = cross(p2 - p0, p3 - p0) / 6.0f;
    const float3 g2 = cross(p3 - p0, p1 - p0) / 6.0f;
    const float3 g3 = cross(p1 - p0, p2 - p0) / 6.0f;
    const float3 g0 = -(g1 + g2 + g3);
    const float w0 = g_inverse_masses[ids.x];
    const float w1 = g_inverse_masses[ids.y];
    const float w2 = g_inverse_masses[ids.z];
    const float w3 = g_inverse_masses[ids.w];
    const float alpha = tet.compliance / max(delta_time * delta_time, 1.0e-8f);
    const float denominator = w0 * dot(g0,g0) + w1 * dot(g1,g1)
        + w2 * dot(g2,g2) + w3 * dot(g3,g3) + alpha;
    const float old_lambda = g_tet_lambda_in[index];
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(denominator, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;
    g_tet_lambdas[index] = old_lambda + scaled_lambda;
    g_tet_corrections[index * 4 + 0] = float4(w0 * g0 * scaled_lambda, 0.0f);
    g_tet_corrections[index * 4 + 1] = float4(w1 * g1 * scaled_lambda, 0.0f);
    g_tet_corrections[index * 4 + 2] = float4(w2 * g2 * scaled_lambda, 0.0f);
    g_tet_corrections[index * 4 + 3] = float4(w3 * g3 * scaled_lambda, 0.0f);
}
