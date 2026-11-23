struct Particle
{
    float4 position;
    float4 velocity;
};

struct BendingConstraint
{
    uint4 indices;
    float rest_distance;
    float compliance;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<BendingConstraint> g_hinges : register(t1);
RWStructuredBuffer<float3> g_corrections : register(u0);
RWStructuredBuffer<float> g_lambdas : register(u1);

cbuffer BendingConstants : register(b0)
{
    uint hinge_count;
    uint edge_count;
    uint area_count;
    float delta_time;
    float relaxation;
    float compliance;
    float padding1;
    float padding2;
};

[numthreads(128, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint hinge_index = dispatch_id.x;
    if (hinge_index >= hinge_count) return;

    const BendingConstraint hinge = g_hinges[hinge_index];
    const Particle opposite_a = g_particles[hinge.indices.z];
    const Particle opposite_b = g_particles[hinge.indices.w];
    const float3 delta = opposite_a.position.xyz - opposite_b.position.xyz;
    const float current_distance = length(delta);
    const float old_lambda = g_lambdas[hinge_index];
    const uint correction_base = edge_count + area_count * 3u + hinge_index * 2u;

    if (current_distance < 1.0e-8f || hinge.rest_distance < 1.0e-8f)
    {
        g_lambdas[hinge_index] = old_lambda;
        g_corrections[correction_base + 0u] = 0.0f;
        g_corrections[correction_base + 1u] = 0.0f;
        return;
    }

    const float constraint = current_distance - hinge.rest_distance;
    const float alpha = compliance / max(delta_time * delta_time, 1.0e-8f);
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(2.0f + alpha, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;
    const float3 gradient = delta / current_distance;

    g_lambdas[hinge_index] = old_lambda + scaled_lambda;
    g_corrections[correction_base + 0u] = gradient * scaled_lambda;
    g_corrections[correction_base + 1u] = -gradient * scaled_lambda;
}
