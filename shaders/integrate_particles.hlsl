struct Particle { float4 position; float4 velocity; };
StructuredBuffer<Particle> g_particles_in : register(t0);
RWStructuredBuffer<Particle> g_particles_out : register(u0);
cbuffer IntegrateConstants : register(b0)
{
    uint particle_count;
    float delta_time;
    float damping;
    float3 gravity;
    float padding;
};
[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= particle_count) return;
    Particle particle = g_particles_in[index];
    particle.velocity.xyz += gravity * delta_time;
    particle.velocity.xyz *= exp(-damping * delta_time);
    particle.position.xyz += particle.velocity.xyz * delta_time;
    g_particles_out[index] = particle;
}
