struct Particle
{
    float4 position;
    float4 velocity;
};

struct AreaConstraint
{
    uint3 indices;
    float rest_area;
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
    float alpha;
    float relaxation;
    float padding;
};

[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint area_index = dispatch_id.x;
    if (area_index >= area_count) return;

    const AreaConstraint area = g_areas[area_index];
    const float3 position_a = g_particles[area.indices.x].position.xyz;
    const float3 position_b = g_particles[area.indices.y].position.xyz;
    const float3 position_c = g_particles[area.indices.z].position.xyz;

    const float3 edge_ab = position_b - position_a;
    const float3 edge_ac = position_c - position_a;
    const float3 edge_bc = position_c - position_b;
    const float3 area_vector = cross(edge_ab, edge_ac);
    const float twice_area_squared = dot(area_vector, area_vector);
    const float old_lambda = g_lambda_in[area_index];

    if (twice_area_squared < 1.0e-16f || area.rest_area < 1.0e-10f)
    {
        g_lambda_out[area_index] = old_lambda;
        const uint correction_index = edge_count + area_index * 3u;
        g_corrections[correction_index + 0u] = 0.0f;
        g_corrections[correction_index + 1u] = 0.0f;
        g_corrections[correction_index + 2u] = 0.0f;
        return;
    }

    const float inverse_twice_area = rsqrt(twice_area_squared);
    const float twice_area = twice_area_squared * inverse_twice_area;
    const float3 normal = area_vector * inverse_twice_area;
    const float3 gradient_a = -0.5f * cross(edge_bc, normal);
    const float3 gradient_b = 0.5f * cross(edge_ac, normal);
    const float3 gradient_c = -0.5f * cross(edge_ab, normal);

    const float constraint = 0.5f * twice_area - area.rest_area;

    // The triangle normal is perpendicular to every edge, so
    // |cross(edge, normal)|^2 == |edge|^2 for the unit normal.
    const float denominator = 0.25f * (
        dot(edge_ab, edge_ab) + dot(edge_ac, edge_ac) + dot(edge_bc, edge_bc)) + alpha;
    const float delta_lambda = (-constraint - alpha * old_lambda) / max(denominator, 1.0e-9f);
    const float scaled_lambda = delta_lambda * relaxation;
    const uint correction_index = edge_count + area_index * 3u;

    g_lambda_out[area_index] = old_lambda + scaled_lambda;
    g_corrections[correction_index + 0u] = gradient_a * scaled_lambda;
    g_corrections[correction_index + 1u] = gradient_b * scaled_lambda;
    g_corrections[correction_index + 2u] = gradient_c * scaled_lambda;
}
