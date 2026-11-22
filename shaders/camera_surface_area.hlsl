struct Particle
{
    float4 position;
    float4 velocity;
};

struct AreaConstraint
{
    uint3 indices;
    float rest_area;
    float compliance;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<AreaConstraint> g_areas : register(t1);
StructuredBuffer<float> g_lambda_in : register(t2);
RWStructuredBuffer<float3> g_corrections : register(u0);
RWStructuredBuffer<float> g_lambda_out : register(u1);

cbuffer AreaConstants : register(b0)
{
    uint area_count;
    uint edge_count;
    float delta_time;
    float relaxation;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint area_index = dispatch_id.x;
    if (area_index >= area_count) return;

    const AreaConstraint area = g_areas[area_index];
    const Particle particle_a = g_particles[area.indices.x];
    const Particle particle_b = g_particles[area.indices.y];
    const Particle particle_c = g_particles[area.indices.z];

    const float3 edge_ab = particle_b.position.xyz - particle_a.position.xyz;
    const float3 edge_ac = particle_c.position.xyz - particle_a.position.xyz;
    const float3 area_vector = cross(edge_ab, edge_ac);
    const float twice_area = length(area_vector);
    const float old_lambda = g_lambda_in[area_index];

    if (twice_area < 1.0e-8f || area.rest_area < 1.0e-10f)
    {
        g_lambda_out[area_index] = old_lambda;
        g_corrections[edge_count + area_index * 3u + 0u] = 0.0f;
        g_corrections[edge_count + area_index * 3u + 1u] = 0.0f;
        g_corrections[edge_count + area_index * 3u + 2u] = 0.0f;
        return;
    }

    const float3 normal = area_vector / twice_area;
    const float3 gradient_a = 0.5f * cross(particle_b.position.xyz - particle_c.position.xyz, normal);
    const float3 gradient_b = 0.5f * cross(particle_c.position.xyz - particle_a.position.xyz, normal);
    const float3 gradient_c = 0.5f * cross(particle_a.position.xyz - particle_b.position.xyz, normal);

    const float current_area = 0.5f * twice_area;
    const float constraint = current_area - area.rest_area;
    const float alpha = area.compliance / max(delta_time * delta_time, 1.0e-8f);
    const float denominator = dot(gradient_a, gradient_a)
        + dot(gradient_b, gradient_b) + dot(gradient_c, gradient_c) + alpha;
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(denominator, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;

    g_lambda_out[area_index] = old_lambda + scaled_lambda;
    g_corrections[edge_count + area_index * 3u + 0u] = gradient_a * scaled_lambda;
    g_corrections[edge_count + area_index * 3u + 1u] = gradient_b * scaled_lambda;
    g_corrections[edge_count + area_index * 3u + 2u] = gradient_c * scaled_lambda;
}
