Texture2D SourceTexture : register(t0);
SamplerState SourceSampler : register(s0);

cbuffer ScaleConstants : register(b0)
{
    float4 SourceRectangle;
};

struct ScaleVertexOutput
{
    float4 position : SV_Position;
    float2 uv : TEXCOORD0;
};

ScaleVertexOutput ScaleVertex(uint vertexId : SV_VertexID)
{
    ScaleVertexOutput output;
    output.uv = float2((vertexId << 1) & 2, vertexId & 2);
    output.position = float4(output.uv.x * 2.0 - 1.0,
        1.0 - output.uv.y * 2.0, 0.0, 1.0);
    return output;
}

float4 ScalePixel(ScaleVertexOutput input) : SV_Target
{
    const float2 sourceUv = SourceRectangle.xy + input.uv * SourceRectangle.zw;
    return SourceTexture.SampleLevel(SourceSampler, sourceUv, 0.0);
}
