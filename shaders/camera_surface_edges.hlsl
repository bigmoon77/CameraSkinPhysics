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
StructuredBuffer<float> g_lambda_in : register(t2);
RWStructuredBuffer<float4> g_edge_corrections : register(u0);
RWStructuredBuffer<float> g_lambda_out : register(u1);

cbuffer EdgeConstants : register(b0)
{
    uint edge_count;
    float delta_time;
    float relaxation;
    float padding;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint edge_index = dispatch_id.x;
    if (edge_index >= edge_count) return;

    const SurfaceEdge edge = g_edges[edge_index];
    const Particle particle_a = g_particles[edge.indices.x];
    const Particle particle_b = g_particles[edge.indices.y];
    const float3 delta = particle_a.position.xyz - particle_b.position.xyz;
    const float current_length = length(delta);

    const float old_lambda = g_lambda_in[edge_index];
    if (current_length < 1.0e-8f || edge.rest_length < 1.0e-8f)
    {
        g_lambda_out[edge_index] = old_lambda;
        g_edge_corrections[edge_index * 2 + 0] = float4(0.0f, 0.0f, 0.0f, 0.0f);
        g_edge_corrections[edge_index * 2 + 1] = float4(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    const float constraint = current_length - edge.rest_length;
    const float alpha = edge.compliance / max(delta_time * delta_time, 1.0e-8f);
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(2.0f + alpha, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;
    const float3 gradient = delta / current_length;
    const float3 correction = gradient * scaled_lambda;

    g_lambda_out[edge_index] = old_lambda + scaled_lambda;
    g_edge_corrections[edge_index * 2 + 0] = float4(correction, 0.0f);
    g_edge_corrections[edge_index * 2 + 1] = float4(-correction, 0.0f);
}
