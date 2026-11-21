struct Particle
{
    float4 position;
    float4 velocity;
};

struct Tetrahedron
{
    uint4 indices;
    float rest_volume;
    float compliance;
    float lambda;
    float padding;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<Tetrahedron> g_tetrahedra : register(t1);
StructuredBuffer<float> g_inverse_masses : register(t2);
StructuredBuffer<float> g_tet_lambda_in : register(t3);
StructuredBuffer<float4> g_rest_positions : register(t4);
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
    const float3 p[4] =
    {
        g_particles[ids.x].position.xyz,
        g_particles[ids.y].position.xyz,
        g_particles[ids.z].position.xyz,
        g_particles[ids.w].position.xyz
    };
    const float3 rest_p[4] =
    {
        g_rest_positions[ids.x].xyz,
        g_rest_positions[ids.y].xyz,
        g_rest_positions[ids.z].xyz,
        g_rest_positions[ids.w].xyz
    };
    const float inverse_mass[4] =
    {
        g_inverse_masses[ids.x],
        g_inverse_masses[ids.y],
        g_inverse_masses[ids.z],
        g_inverse_masses[ids.w]
    };

    float3 correction[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // XPBD tetrahedron volume constraint.
    const float constraint = signed_volume(p[0], p[1], p[2], p[3]) - tet.rest_volume;
    const float3 gradient1 = cross(p[2] - p[0], p[3] - p[0]) / 6.0f;
    const float3 gradient2 = cross(p[3] - p[0], p[1] - p[0]) / 6.0f;
    const float3 gradient3 = cross(p[1] - p[0], p[2] - p[0]) / 6.0f;
    const float3 gradient0 = -(gradient1 + gradient2 + gradient3);
    const float3 volume_gradients[4] = {gradient0, gradient1, gradient2, gradient3};

    const float alpha = tet.compliance / max(delta_time * delta_time, 1.0e-8f);
    float denominator = alpha;
    for (uint corner = 0; corner < 4; ++corner)
        denominator += inverse_mass[corner] * dot(volume_gradients[corner], volume_gradients[corner]);

    const uint lambda_base = index * 7;
    const float old_volume_lambda = g_tet_lambda_in[lambda_base];
    const float delta_volume_lambda = (-constraint - alpha * old_volume_lambda)
        / max(denominator, 1.0e-9f);
    const float scaled_volume_lambda = delta_volume_lambda * relaxation;
    g_tet_lambdas[lambda_base] = old_volume_lambda + scaled_volume_lambda;

    for (uint corner = 0; corner < 4; ++corner)
        correction[corner] += inverse_mass[corner] * volume_gradients[corner] * scaled_volume_lambda;

    // Six edge-distance constraints prevent tetrahedra from shearing while retaining volume.
    const uint2 edge_corners[6] =
    {
        uint2(0, 1), uint2(0, 2), uint2(0, 3),
        uint2(1, 2), uint2(1, 3), uint2(2, 3)
    };
    const float edge_alpha = max(tet.compliance, 1.0e-8f) / max(delta_time * delta_time, 1.0e-8f);
    for (uint edge = 0; edge < 6; ++edge)
    {
        const uint a = edge_corners[edge].x;
        const uint b = edge_corners[edge].y;
        const float3 rest_delta = rest_p[a] - rest_p[b];
        const float rest_length = length(rest_delta);
        const float3 delta = p[a] - p[b];
        const float current_length = length(delta);
        if (current_length < 1.0e-8f || rest_length < 1.0e-8f)
        {
            g_tet_lambdas[lambda_base + edge + 1] = g_tet_lambda_in[lambda_base + edge + 1];
            continue;
        }

        const float constraint_length = current_length - rest_length;
        const float3 gradient = delta / current_length;
        const float edge_denominator = inverse_mass[a] + inverse_mass[b] + edge_alpha;
        const float old_edge_lambda = g_tet_lambda_in[lambda_base + edge + 1];
        const float delta_edge_lambda = (-constraint_length - edge_alpha * old_edge_lambda)
            / max(edge_denominator, 1.0e-9f);
        const float scaled_edge_lambda = delta_edge_lambda * relaxation;
        g_tet_lambdas[lambda_base + edge + 1] = old_edge_lambda + scaled_edge_lambda;
        correction[a] += inverse_mass[a] * gradient * scaled_edge_lambda;
        correction[b] -= inverse_mass[b] * gradient * scaled_edge_lambda;
    }

    g_tet_corrections[index * 4 + 0] = float4(correction[0], 0.0f);
    g_tet_corrections[index * 4 + 1] = float4(correction[1], 0.0f);
    g_tet_corrections[index * 4 + 2] = float4(correction[2], 0.0f);
    g_tet_corrections[index * 4 + 3] = float4(correction[3], 0.0f);
}
