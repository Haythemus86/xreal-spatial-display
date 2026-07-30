cbuffer CameraConstants : register(b0)
{
    row_major float4x4 viewProjection;
    float4 desktopContentBounds;
    float4 desktopCrop;
    uint desktopRotation;
    uint desktopFlipY;
    uint desktopEnabled;
    uint desktopUnavailable;
    uint desktopBackgroundGrid;
    uint3 desktopPadding;
    uint desktopDebugMode;
    uint3 desktopDebugPadding;
};

Texture2D desktopTexture : register(t0);
SamplerState desktopSampler : register(s0);

struct VertexInput
{
    float3 position : POSITION;
    float4 color : COLOR;
    float2 texcoord : TEXCOORD;
};

struct PixelInput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
    float2 texcoord : TEXCOORD;
};

PixelInput VSMain(VertexInput input)
{
    PixelInput output;
    output.position = mul(viewProjection, float4(input.position, 1.0));
    output.color = input.color;
    output.texcoord = input.texcoord;
    return output;
}

float4 PSMain(PixelInput input) : SV_TARGET
{
    if (desktopDebugMode == 1)
    {
        return float4(1.0, 0.0, 0.0, 1.0);
    }
    if (desktopDebugMode == 2)
    {
        return float4(input.texcoord.x, input.texcoord.y, 0.0, 1.0);
    }
    if (desktopDebugMode == 3 || desktopDebugMode == 4)
    {
        return float4(desktopTexture.Sample(desktopSampler, input.texcoord).rgb, 1.0);
    }
    if (desktopEnabled != 0)
    {
        if (desktopUnavailable != 0)
        {
            const float stripe = fmod(floor(input.texcoord.x * 24.0)
                + floor(input.texcoord.y * 14.0), 2.0);
            return lerp(float4(0.12, 0.015, 0.02, 1.0),
                        float4(0.28, 0.035, 0.04, 1.0), stripe);
        }
        if (input.texcoord.x < desktopContentBounds.x
            || input.texcoord.y < desktopContentBounds.y
            || input.texcoord.x > desktopContentBounds.z
            || input.texcoord.y > desktopContentBounds.w)
        {
            if (desktopBackgroundGrid != 0)
            {
                const float gridLine = max(
                    step(0.92, frac(input.texcoord.x * 20.0)),
                    step(0.92, frac(input.texcoord.y * 12.0)));
                return lerp(float4(0.0, 0.0, 0.0, 1.0),
                            float4(0.05, 0.09, 0.14, 1.0), gridLine);
            }
            return float4(0.0, 0.0, 0.0, 1.0);
        }
        float2 oriented = (input.texcoord - desktopContentBounds.xy)
            / (desktopContentBounds.zw - desktopContentBounds.xy);
        oriented = desktopCrop.xy + oriented * desktopCrop.zw;
        if (desktopFlipY != 0)
        {
            oriented.y = 1.0 - oriented.y;
        }
        float2 source = oriented;
        if (desktopRotation == 90)
        {
            source = float2(oriented.y, 1.0 - oriented.x);
        }
        else if (desktopRotation == 180)
        {
            source = 1.0 - oriented;
        }
        else if (desktopRotation == 270)
        {
            source = float2(1.0 - oriented.y, oriented.x);
        }
        float4 desktopColor = desktopTexture.Sample(desktopSampler, source);
        // Desktop Duplication does not guarantee meaningful alpha. The current
        // SRC_ALPHA blend state requires an opaque panel base to remain visible.
        desktopColor.a = 1.0;
        return desktopColor;
    }
    return input.color;
}

float4 PSOverlay(PixelInput input) : SV_TARGET
{
    return input.color;
}
