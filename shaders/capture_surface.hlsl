struct VertexInput { float3 position : POSITION; float3 normal : NORMAL; };
struct PixelInput { float4 position : SV_Position; float3 world_position : TEXCOORD0; float3 world_normal : TEXCOORD1; };
cbuffer CaptureConstants : register(b0)
{
    row_major float4x4 world;
    row_major float4x4 view_projection;
};
PixelInput VSMain(VertexInput input)
{
    PixelInput output;
    const float4 world_position = mul(float4(input.position, 1.0f), world);
    output.position = mul(world_position, view_projection);
    output.world_position = world_position.xyz;
    output.world_normal = normalize(mul(float4(input.normal, 0.0f), world).xyz);
    return output;
}
float4 PSMain(PixelInput input) : SV_Target0
{
    return float4(normalize(input.world_normal) * 0.5f + 0.5f, 1.0f);
}
