struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};
struct PixelInput
{
    float4 position : SV_Position;
    float3 normal : TEXCOORD0;
};
cbuffer DrawConstants : register(b0)
{
    row_major float4x4 view_projection;
    row_major float4x4 world;
    float4 base_color;
};
PixelInput VSMain(VertexInput input)
{
    PixelInput output;
    const float4 world_position = mul(float4(input.position, 1.0f), world);
    output.position = mul(world_position, view_projection);
    output.normal = normalize(mul(float4(input.normal, 0.0f), world).xyz);
    return output;
}
float4 PSMain(PixelInput input) : SV_Target0
{
    const float3 light_direction = normalize(float3(-0.4f, 0.7f, -0.6f));
    const float diffuse = saturate(dot(normalize(input.normal), light_direction));
    return float4(base_color.rgb * (0.18f + diffuse * 0.82f), base_color.a);
}
