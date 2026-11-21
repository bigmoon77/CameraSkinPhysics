struct Particle { float4 position; float4 velocity; };
StructuredBuffer<Particle> g_particles_in : register(t0);
StructuredBuffer<uint2> g_incidence_ranges : register(t1);
StructuredBuffer<uint> g_incident_tetrahedra : register(t2);
StructuredBuffer<float4> g_tet_corrections : register(t3);
StructuredBuffer<uint4> g_tetrahedron_indices : register(t4);
RWStructuredBuffer<Particle> g_particles_out : register(u0);
cbuffer GatherConstants : register(b0)
{
    uint particle_count;
    float inverse_mass;
    float max_correction;
    float padding;
};
[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint particle_index = dispatch_id.x;
    if (particle_index >= particle_count) return;
    Particle particle = g_particles_in[particle_index];
    const uint2 range = g_incidence_ranges[particle_index];
    float3 correction = 0.0f;
    for (uint i = 0; i < range.y; ++i)
    {
        const uint tet_index = g_incident_tetrahedra[range.x + i];
        const uint4 indices = g_tetrahedron_indices[tet_index];
        const float3 tet_correction = g_tet_corrections[tet_index].xyz;
        if (indices.x == particle_index) correction += tet_correction;
        if (indices.y == particle_index) correction += tet_correction;
        if (indices.z == particle_index) correction += tet_correction;
        if (indices.w == particle_index) correction += tet_correction;
    }
    const float correction_length = length(correction);
    if (correction_length > max_correction) correction *= max_correction / correction_length;
    particle.position.xyz += correction * inverse_mass;
    g_particles_out[particle_index] = particle;
}
