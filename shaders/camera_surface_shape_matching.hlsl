struct Particle
{
    float4 position;
    float4 velocity;
};

struct ClusterTransform
{
    float4 row0;
    float4 row1;
    float4 row2;
};

StructuredBuffer<Particle> g_particles : register(t0);
StructuredBuffer<float4> g_rest_positions : register(t1);
StructuredBuffer<uint2> g_cluster_vertex_ranges : register(t2);
StructuredBuffer<uint> g_cluster_vertices : register(t3);
RWStructuredBuffer<ClusterTransform> g_cluster_transforms : register(u0);

cbuffer ShapeConstants : register(b0)
{
    uint element_count;
    float strength;
    float max_correction;
    float delta_time;
};

[numthreads(64, 1, 1)]
void build_transforms(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint cluster_index = dispatch_id.x;
    if (cluster_index >= element_count) return;

    const uint2 range = g_cluster_vertex_ranges[cluster_index];
    if (range.y == 0u) return;

    float3 rest_center = 0.0f;
    float3 current_center = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint vertex_index = g_cluster_vertices[range.x + offset];
        rest_center += g_rest_positions[vertex_index].xyz;
        current_center += g_particles[vertex_index].position.xyz;
    }
    const float inverse_count = rcp((float)range.y);
    rest_center *= inverse_count;
    current_center *= inverse_count;

    float3 row0 = 0.0f;
    float3 row1 = 0.0f;
    float3 row2 = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint vertex_index = g_cluster_vertices[range.x + offset];
        const float3 rest = g_rest_positions[vertex_index].xyz - rest_center;
        const float3 current = g_particles[vertex_index].position.xyz - current_center;
        row0 += current.x * rest;
        row1 += current.y * rest;
        row2 += current.z * rest;
    }

    const float covariance_length = sqrt(dot(row0, row0) + dot(row1, row1) + dot(row2, row2));
    if (covariance_length > 1.0e-10f)
    {
        const float inverse_length = rcp(covariance_length);
        row0 *= inverse_length;
        row1 *= inverse_length;
        row2 *= inverse_length;
        row0.x += 1.0e-4f;
        row1.y += 1.0e-4f;
        row2.z += 1.0e-4f;

        const float3 column0 = float3(row0.x, row1.x, row2.x);
        const float3 column1 = float3(row0.y, row1.y, row2.y);
        const float3 column2 = float3(row0.z, row1.z, row2.z);
        const float covariance_determinant = dot(row0, cross(row1, row2));
        if (covariance_determinant < 0.0f)
        {
            const float length0 = dot(column0, column0);
            const float length1 = dot(column1, column1);
            const float length2 = dot(column2, column2);
            if (length0 <= length1 && length0 <= length2)
            {
                row0.x = -row0.x;
                row1.x = -row1.x;
                row2.x = -row2.x;
            }
            else if (length1 <= length2)
            {
                row0.y = -row0.y;
                row1.y = -row1.y;
                row2.y = -row2.y;
            }
            else
            {
                row0.z = -row0.z;
                row1.z = -row1.z;
                row2.z = -row2.z;
            }
        }

        bool rotation_valid = true;
        for (uint iteration = 0; iteration < 12u; ++iteration)
        {
            const float3 cofactor0 = cross(row1, row2);
            const float3 cofactor1 = cross(row2, row0);
            const float3 cofactor2 = cross(row0, row1);
            const float determinant = dot(row0, cofactor0);
            if (abs(determinant) < 1.0e-8f)
            {
                rotation_valid = false;
                break;
            }

            row0 = 0.5f * (row0 + cofactor0 / determinant);
            row1 = 0.5f * (row1 + cofactor1 / determinant);
            row2 = 0.5f * (row2 + cofactor2 / determinant);
        }
        if (!rotation_valid)
        {
            row0 = float3(1.0f, 0.0f, 0.0f);
            row1 = float3(0.0f, 1.0f, 0.0f);
            row2 = float3(0.0f, 0.0f, 1.0f);
        }
    }
    else
    {
        row0 = float3(1.0f, 0.0f, 0.0f);
        row1 = float3(0.0f, 1.0f, 0.0f);
        row2 = float3(0.0f, 0.0f, 1.0f);
    }

    const float3 translation = current_center - float3(
        dot(row0, rest_center), dot(row1, rest_center), dot(row2, rest_center));
    ClusterTransform transform;
    transform.row0 = float4(row0, translation.x);
    transform.row1 = float4(row1, translation.y);
    transform.row2 = float4(row2, translation.z);
    g_cluster_transforms[cluster_index] = transform;
}

StructuredBuffer<uint2> g_vertex_cluster_ranges : register(t4);
StructuredBuffer<uint> g_vertex_clusters : register(t5);
StructuredBuffer<ClusterTransform> g_transforms : register(t6);
RWStructuredBuffer<Particle> g_particles_out : register(u1);


[numthreads(256, 1, 1)]
void apply_matching(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_id.x;
    if (vertex_index >= element_count) return;

    Particle particle = g_particles[vertex_index];
    const uint2 range = g_vertex_cluster_ranges[vertex_index];
    if (range.y == 0u || strength <= 0.0f)
    {
        g_particles_out[vertex_index] = particle;
        return;
    }

    const float3 rest_position = g_rest_positions[vertex_index].xyz;
    float3 target = 0.0f;
    for (uint offset = 0; offset < range.y; ++offset)
    {
        const uint cluster_index = g_vertex_clusters[range.x + offset];
        const ClusterTransform transform = g_transforms[cluster_index];
        target += float3(
            dot(transform.row0.xyz, rest_position) + transform.row0.w,
            dot(transform.row1.xyz, rest_position) + transform.row1.w,
            dot(transform.row2.xyz, rest_position) + transform.row2.w);
    }
    target /= (float)range.y;

    float3 correction = (target - particle.position.xyz) * saturate(strength);
    const float correction_length = length(correction);
    if (correction_length > max_correction && correction_length > 1.0e-8f)
        correction *= max_correction / correction_length;

    particle.position.xyz += correction;
    particle.velocity.xyz += correction / max(delta_time, 1.0e-6f);
    g_particles_out[vertex_index] = particle;
}
