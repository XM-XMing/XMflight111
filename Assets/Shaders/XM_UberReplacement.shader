// filename: Assets/Shaders/XM_UberReplacement.shader
Shader "Hidden/XM_UberReplacement"
{
    Properties {
        _MinDepth ("Min Depth Range", Float)    = 0.3
        _MaxDepth ("Max Depth Range", Float)    = 6.0
        _MainTex  ("Base (RGB)",      2D)       = "white" {}
        _Cutoff   ("Alpha Cutoff", Range(0,1))  = 0.5
    }

    SubShader {
        Tags { "RenderType" = "Opaque" }

        Pass {
            ZWrite On  ZTest LEqual  Cull Back

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            uniform float _MinDepth;
            uniform float _MaxDepth;

            struct v2f {
                float4 pos   : SV_POSITION;
                float  depth : TEXCOORD0;
            };

            v2f vert(appdata_base v) {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                COMPUTE_EYEDEPTH(o.depth);
                return o;
            }

            float4 frag(v2f i) : SV_Target {
                float d = i.depth;
                return (d >= _MinDepth && d <= _MaxDepth)
                     ? float4(d, 0, 0, 1)
                     : float4(0, 0, 0, 1);
            }
            ENDCG
        }
    }

    SubShader {
        Tags { "RenderType" = "TransparentCutout" }

        Pass {
            ZWrite On  ZTest LEqual  Cull Back

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            uniform float     _MinDepth;
            uniform float     _MaxDepth;
            uniform float     _Cutoff;
            uniform sampler2D _MainTex;
            float4 _MainTex_ST;

            struct v2f {
                float4 pos   : SV_POSITION;
                float2 uv    : TEXCOORD0;
                float  depth : TEXCOORD1;
            };

            v2f vert(appdata_base v) {
                v2f o;
                o.pos   = UnityObjectToClipPos(v.vertex);
                o.uv    = TRANSFORM_TEX(v.texcoord, _MainTex);
                COMPUTE_EYEDEPTH(o.depth);
                return o;
            }

            float4 frag(v2f i) : SV_Target {
                clip(tex2D(_MainTex, i.uv).a - _Cutoff);
                float d = i.depth;
                return (d >= _MinDepth && d <= _MaxDepth)
                     ? float4(d, 0, 0, 1)
                     : float4(0, 0, 0, 1);
            }
            ENDCG
        }
    }

    Fallback Off
}
