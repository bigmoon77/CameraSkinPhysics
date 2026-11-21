RWStructuredBuffer<float> g_lambdas : register(u0);
cbuffer ClearConstants : register(b0)
{
    uint element_count;
    float3 padding;
};
[numthreads(256, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    const uint index = dispatch_id.x;
    if (index >= element_count) return;
    g_lambdas[index] = 0.0f;
}
