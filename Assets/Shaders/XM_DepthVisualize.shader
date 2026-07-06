// filename: Assets/Shaders/XM_DepthVisualize.shader
Shader "Hidden/XM_DepthVisualize"
{
    Properties {
        _MainTex ("Depth", 2D) = "black" {}
        _MaxDepth ("Max Depth", Float) = 6.0
    }

    SubShader {
        Cull Off
        ZWrite Off
        ZTest Always

        Pass {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            uniform float _MaxDepth;

            float4 frag(v2f_img i) : SV_Target {
                float d = tex2D(_MainTex, i.uv).r;
                if (d <= 0) return float4(0, 0, 0, 1);
                float v = 1.0 - saturate(d / _MaxDepth);
                return float4(v, v, v, 1);
            }
            ENDCG
        }
    }

    Fallback Off
}
