struct Particle
{
    float4 position;
    float4 velocity;
};

struct SurfaceEdge
{
    uint2 indices;
    float rest_length;
    float compliance;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<SurfaceEdge> g_edges : register(t1);
RWStructuredBuffer<float3> g_edge_corrections : register(u0);
RWStructuredBuffer<float> g_lambdas : register(u1);

cbuffer EdgeConstants : register(b0)
{
    uint edge_count;
    float alpha;
    float relaxation;
    float padding;
};

[numthreads(128, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint edge_index = dispatch_id.x;
    if (edge_index >= edge_count) return;

    const SurfaceEdge edge = g_edges[edge_index];
    const float3 delta = g_particles[edge.indices.x].position.xyz
        - g_particles[edge.indices.y].position.xyz;
    const float length_squared = dot(delta, delta);
    const float old_lambda = g_lambdas[edge_index];

    if (length_squared < 1.0e-16f || edge.rest_length < 1.0e-8f)
    {
        g_lambdas[edge_index] = old_lambda;
        g_edge_corrections[edge_index] = 0.0f;
        return;
    }

    const float inverse_length = rsqrt(length_squared);
    const float current_length = length_squared * inverse_length;
    const float constraint = current_length - edge.rest_length;
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(2.0f + alpha, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;

    g_lambdas[edge_index] = old_lambda + scaled_lambda;
    g_edge_corrections[edge_index] = delta * (inverse_length * scaled_lambda);
}
