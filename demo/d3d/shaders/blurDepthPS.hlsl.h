#if 0
//
// Generated by Microsoft (R) HLSL Shader Compiler 10.1
//
//
// Buffer Definitions: 
//
// cbuffer constBuf
// {
//
//   struct FluidShaderConst
//   {
//       
//       float4x4 modelViewProjection;  // Offset:    0
//       float4x4 modelView;            // Offset:   64
//       float4x4 projection;           // Offset:  128
//       float4x4 inverseModelView;     // Offset:  192
//       float4x4 inverseProjection;    // Offset:  256
//       float4 invTexScale;            // Offset:  320
//       float3 invViewport;            // Offset:  336
//       float _pad0;                   // Offset:  348
//       float blurRadiusWorld;         // Offset:  352
//       float blurScale;               // Offset:  356
//       float blurFalloff;             // Offset:  360
//       int debug;                     // Offset:  364
//       float3 lightPos;               // Offset:  368
//       float _pad1;                   // Offset:  380
//       float3 lightDir;               // Offset:  384
//       float _pad2;                   // Offset:  396
//       float4x4 lightTransform;       // Offset:  400
//       float4 color;                  // Offset:  464
//       float4 clipPosToEye;           // Offset:  480
//       float spotMin;                 // Offset:  496
//       float spotMax;                 // Offset:  500
//       float ior;                     // Offset:  504
//       float pointRadius;             // Offset:  508
//       float4 shadowTaps[12];         // Offset:  512
//
//   } gParams;                         // Offset:    0 Size:   704
//
// }
//
//
// Resource Bindings:
//
// Name                                 Type  Format         Dim      HLSL Bind  Count
// ------------------------------ ---------- ------- ----------- -------------- ------
// depthTex                          texture   float          2d             t0      1 
// constBuf                          cbuffer      NA          NA            cb0      1 
//
//
//
// Input signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_POSITION              0   xyzw        0      POS   float   xy  
// TEXCOORD                 0   xy          1     NONE   float       
//
//
// Output signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_TARGET                0   x           0   TARGET   float   x   
//
ps_5_0
dcl_globalFlags refactoringAllowed
dcl_constantbuffer CB0[23], immediateIndexed
dcl_resource_texture2d (float,float,float,float) t0
dcl_input_ps_siv linear noperspective v0.xy, position
dcl_output o0.x
dcl_temps 5
ftoi r0.xy, v0.xyxx
mov r0.zw, l(0,0,0,0)
ld_indexable(texture2d)(float,float,float,float) r0.x, r0.xyzw, t0.xyzw
div r0.y, cb0[22].x, -r0.x
mul r0.y, r0.y, cb0[22].y
min r0.y, r0.y, l(5.000000)
div r0.z, l(1.000000, 1.000000, 1.000000, 1.000000), r0.y
round_pi r0.w, r0.y
add r1.x, -r0.y, r0.w
mov r2.zw, l(0,0,0,0)
mov r1.yzw, l(0,0,0,0)
mov r3.y, -r0.w
loop 
  lt r3.z, r0.w, r3.y
  breakc_nz r3.z
  mov r4.xyz, r1.yzwy
  mov r4.w, -r0.w
  loop 
    lt r3.z, r0.w, r4.w
    breakc_nz r3.z
    mov r3.x, r4.w
    add r3.zw, r3.xxxy, v0.xxxy
    ftoi r2.xy, r3.zwzz
    ld_indexable(texture2d)(float,float,float,float) r2.x, r2.xyzw, t0.xyzw
    dp2 r2.y, r3.xyxx, r3.xyxx
    sqrt r2.y, r2.y
    mul r2.y, r0.z, r2.y
    mul r2.y, r2.y, r2.y
    mul r2.y, r2.y, l(-1.442695)
    exp r2.y, r2.y
    add r3.x, -r0.x, r2.x
    mul r3.x, r3.x, l(5.500000)
    mul r3.x, r3.x, r3.x
    mul r3.x, r3.x, l(-1.442695)
    exp r3.x, r3.x
    max r3.z, |r3.y|, |r4.w|
    ge r3.z, r3.z, r0.y
    and r3.z, r3.z, l(0x3f800000)
    mad r3.z, -r3.z, r1.x, l(1.000000)
    mul r2.x, r2.y, r2.x
    mul r2.x, r3.x, r2.x
    mad r4.x, r2.x, r3.z, r4.x
    mul r2.x, r2.y, r3.x
    mad r4.y, r2.x, r3.z, r4.y
    mad r4.z, r3.x, r3.z, r4.z
    add r4.w, r4.w, l(1.000000)
  endloop 
  mov r1.yzw, r4.xxyz
  add r3.y, r3.y, l(1.000000)
endloop 
lt r0.z, l(0.000000), r1.z
div r0.w, r1.y, r1.z
movc r0.z, r0.z, r0.w, r1.y
mad r0.y, r0.y, l(2.000000), l(1.000000)
mul r0.y, r0.y, r0.y
div r0.y, r1.w, r0.y
add r0.z, -r0.x, r0.z
mad o0.x, r0.y, r0.z, r0.x
ret 
// Approximately 59 instruction slots used
#endif

const BYTE g_blurDepthPS[] =
{
     68,  88,  66,  67, 239, 147, 
    201, 219,  80,  87, 165,  38, 
    211,  43, 251,  27,  46, 176, 
    171, 121,   1,   0,   0,   0, 
     32,  12,   0,   0,   5,   0, 
      0,   0,  52,   0,   0,   0, 
    144,   4,   0,   0, 232,   4, 
      0,   0,  28,   5,   0,   0, 
    132,  11,   0,   0,  82,  68, 
     69,  70,  84,   4,   0,   0, 
      1,   0,   0,   0, 144,   0, 
      0,   0,   2,   0,   0,   0, 
     60,   0,   0,   0,   0,   5, 
    255, 255,   0,   1,   0,   0, 
     44,   4,   0,   0,  82,  68, 
     49,  49,  60,   0,   0,   0, 
     24,   0,   0,   0,  32,   0, 
      0,   0,  40,   0,   0,   0, 
     36,   0,   0,   0,  12,   0, 
      0,   0,   0,   0,   0,   0, 
    124,   0,   0,   0,   2,   0, 
      0,   0,   5,   0,   0,   0, 
      4,   0,   0,   0, 255, 255, 
    255, 255,   0,   0,   0,   0, 
      1,   0,   0,   0,   1,   0, 
      0,   0, 133,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   1,   0,   0,   0, 
      1,   0,   0,   0, 100, 101, 
    112, 116, 104,  84, 101, 120, 
      0,  99, 111, 110, 115, 116, 
     66, 117, 102,   0, 171, 171, 
    133,   0,   0,   0,   1,   0, 
      0,   0, 168,   0,   0,   0, 
    192,   2,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    208,   0,   0,   0,   0,   0, 
      0,   0, 192,   2,   0,   0, 
      2,   0,   0,   0,   8,   4, 
      0,   0,   0,   0,   0,   0, 
    255, 255, 255, 255,   0,   0, 
      0,   0, 255, 255, 255, 255, 
      0,   0,   0,   0, 103,  80, 
     97, 114,  97, 109, 115,   0, 
     70, 108, 117, 105, 100,  83, 
    104,  97, 100, 101, 114,  67, 
    111, 110, 115, 116,   0, 109, 
    111, 100, 101, 108,  86, 105, 
    101, 119,  80, 114, 111, 106, 
    101,  99, 116, 105, 111, 110, 
      0, 102, 108, 111,  97, 116, 
     52, 120,  52,   0, 171, 171, 
      3,   0,   3,   0,   4,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 253,   0,   0,   0, 
    109, 111, 100, 101, 108,  86, 
    105, 101, 119,   0, 112, 114, 
    111, 106, 101,  99, 116, 105, 
    111, 110,   0, 105, 110, 118, 
    101, 114, 115, 101,  77, 111, 
    100, 101, 108,  86, 105, 101, 
    119,   0, 105, 110, 118, 101, 
    114, 115, 101,  80, 114, 111, 
    106, 101,  99, 116, 105, 111, 
    110,   0, 105, 110, 118,  84, 
    101, 120,  83,  99,  97, 108, 
    101,   0, 102, 108, 111,  97, 
    116,  52,   0, 171,   1,   0, 
      3,   0,   1,   0,   4,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    112,   1,   0,   0, 105, 110, 
    118,  86, 105, 101, 119, 112, 
    111, 114, 116,   0, 102, 108, 
    111,  97, 116,  51,   0, 171, 
      1,   0,   3,   0,   1,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 168,   1,   0,   0, 
     95, 112,  97, 100,  48,   0, 
    102, 108, 111,  97, 116,   0, 
      0,   0,   3,   0,   1,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 218,   1,   0,   0, 
     98, 108, 117, 114,  82,  97, 
    100, 105, 117, 115,  87, 111, 
    114, 108, 100,   0,  98, 108, 
    117, 114,  83,  99,  97, 108, 
    101,   0,  98, 108, 117, 114, 
     70,  97, 108, 108, 111, 102, 
    102,   0, 100, 101,  98, 117, 
    103,   0, 105, 110, 116,   0, 
      0,   0,   2,   0,   1,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  48,   2,   0,   0, 
    108, 105, 103, 104, 116,  80, 
    111, 115,   0,  95, 112,  97, 
    100,  49,   0, 108, 105, 103, 
    104, 116,  68, 105, 114,   0, 
     95, 112,  97, 100,  50,   0, 
    108, 105, 103, 104, 116,  84, 
    114,  97, 110, 115, 102, 111, 
    114, 109,   0,  99, 111, 108, 
    111, 114,   0,  99, 108, 105, 
    112,  80, 111, 115,  84, 111, 
     69, 121, 101,   0, 115, 112, 
    111, 116,  77, 105, 110,   0, 
    115, 112, 111, 116,  77,  97, 
    120,   0, 105, 111, 114,   0, 
    112, 111, 105, 110, 116,  82, 
     97, 100, 105, 117, 115,   0, 
    115, 104,  97, 100, 111, 119, 
     84,  97, 112, 115,   0, 171, 
      1,   0,   3,   0,   1,   0, 
      4,   0,  12,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 112,   1,   0,   0, 
    233,   0,   0,   0,   8,   1, 
      0,   0,   0,   0,   0,   0, 
     44,   1,   0,   0,   8,   1, 
      0,   0,  64,   0,   0,   0, 
     54,   1,   0,   0,   8,   1, 
      0,   0, 128,   0,   0,   0, 
     65,   1,   0,   0,   8,   1, 
      0,   0, 192,   0,   0,   0, 
     82,   1,   0,   0,   8,   1, 
      0,   0,   0,   1,   0,   0, 
    100,   1,   0,   0, 120,   1, 
      0,   0,  64,   1,   0,   0, 
    156,   1,   0,   0, 176,   1, 
      0,   0,  80,   1,   0,   0, 
    212,   1,   0,   0, 224,   1, 
      0,   0,  92,   1,   0,   0, 
      4,   2,   0,   0, 224,   1, 
      0,   0,  96,   1,   0,   0, 
     20,   2,   0,   0, 224,   1, 
      0,   0, 100,   1,   0,   0, 
     30,   2,   0,   0, 224,   1, 
      0,   0, 104,   1,   0,   0, 
     42,   2,   0,   0,  52,   2, 
      0,   0, 108,   1,   0,   0, 
     88,   2,   0,   0, 176,   1, 
      0,   0, 112,   1,   0,   0, 
     97,   2,   0,   0, 224,   1, 
      0,   0, 124,   1,   0,   0, 
    103,   2,   0,   0, 176,   1, 
      0,   0, 128,   1,   0,   0, 
    112,   2,   0,   0, 224,   1, 
      0,   0, 140,   1,   0,   0, 
    118,   2,   0,   0,   8,   1, 
      0,   0, 144,   1,   0,   0, 
    133,   2,   0,   0, 120,   1, 
      0,   0, 208,   1,   0,   0, 
    139,   2,   0,   0, 120,   1, 
      0,   0, 224,   1,   0,   0, 
    152,   2,   0,   0, 224,   1, 
      0,   0, 240,   1,   0,   0, 
    160,   2,   0,   0, 224,   1, 
      0,   0, 244,   1,   0,   0, 
    168,   2,   0,   0, 224,   1, 
      0,   0, 248,   1,   0,   0, 
    172,   2,   0,   0, 224,   1, 
      0,   0, 252,   1,   0,   0, 
    184,   2,   0,   0, 196,   2, 
      0,   0,   0,   2,   0,   0, 
      5,   0,   0,   0,   1,   0, 
    176,   0,   0,   0,  24,   0, 
    232,   2,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 216,   0,   0,   0, 
     77, 105,  99, 114, 111, 115, 
    111, 102, 116,  32,  40,  82, 
     41,  32,  72,  76,  83,  76, 
     32,  83, 104,  97, 100, 101, 
    114,  32,  67, 111, 109, 112, 
    105, 108, 101, 114,  32,  49, 
     48,  46,  49,   0,  73,  83, 
     71,  78,  80,   0,   0,   0, 
      2,   0,   0,   0,   8,   0, 
      0,   0,  56,   0,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,  15,   3, 
      0,   0,  68,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      1,   0,   0,   0,   3,   0, 
      0,   0,  83,  86,  95,  80, 
     79,  83,  73,  84,  73,  79, 
     78,   0,  84,  69,  88,  67, 
     79,  79,  82,  68,   0, 171, 
    171, 171,  79,  83,  71,  78, 
     44,   0,   0,   0,   1,   0, 
      0,   0,   8,   0,   0,   0, 
     32,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   1,  14,   0,   0, 
     83,  86,  95,  84,  65,  82, 
     71,  69,  84,   0, 171, 171, 
     83,  72,  69,  88,  96,   6, 
      0,   0,  80,   0,   0,   0, 
    152,   1,   0,   0, 106,   8, 
      0,   1,  89,   0,   0,   4, 
     70, 142,  32,   0,   0,   0, 
      0,   0,  23,   0,   0,   0, 
     88,  24,   0,   4,   0, 112, 
     16,   0,   0,   0,   0,   0, 
     85,  85,   0,   0, 100,  32, 
      0,   4,  50,  16,  16,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0, 101,   0,   0,   3, 
     18,  32,  16,   0,   0,   0, 
      0,   0, 104,   0,   0,   2, 
      5,   0,   0,   0,  27,   0, 
      0,   5,  50,   0,  16,   0, 
      0,   0,   0,   0,  70,  16, 
     16,   0,   0,   0,   0,   0, 
     54,   0,   0,   8, 194,   0, 
     16,   0,   0,   0,   0,   0, 
      2,  64,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  45,   0,   0, 137, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  18,   0,  16,   0, 
      0,   0,   0,   0,  70,  14, 
     16,   0,   0,   0,   0,   0, 
     70, 126,  16,   0,   0,   0, 
      0,   0,  14,   0,   0,   9, 
     34,   0,  16,   0,   0,   0, 
      0,   0,  10, 128,  32,   0, 
      0,   0,   0,   0,  22,   0, 
      0,   0,  10,   0,  16, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  56,   0,   0,   8, 
     34,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      0,   0,   0,   0,  26, 128, 
     32,   0,   0,   0,   0,   0, 
     22,   0,   0,   0,  51,   0, 
      0,   7,  34,   0,  16,   0, 
      0,   0,   0,   0,  26,   0, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    160,  64,  14,   0,   0,  10, 
     66,   0,  16,   0,   0,   0, 
      0,   0,   2,  64,   0,   0, 
      0,   0, 128,  63,   0,   0, 
    128,  63,   0,   0, 128,  63, 
      0,   0, 128,  63,  26,   0, 
     16,   0,   0,   0,   0,   0, 
     66,   0,   0,   5, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,   0,   0,   0,   8, 
     18,   0,  16,   0,   1,   0, 
      0,   0,  26,   0,  16, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  54,   0, 
      0,   8, 194,   0,  16,   0, 
      2,   0,   0,   0,   2,  64, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     54,   0,   0,   8, 226,   0, 
     16,   0,   1,   0,   0,   0, 
      2,  64,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  54,   0,   0,   6, 
     34,   0,  16,   0,   3,   0, 
      0,   0,  58,   0,  16, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  48,   0,   0,   1, 
     49,   0,   0,   7,  66,   0, 
     16,   0,   3,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      3,   0,   0,   0,   3,   0, 
      4,   3,  42,   0,  16,   0, 
      3,   0,   0,   0,  54,   0, 
      0,   5, 114,   0,  16,   0, 
      4,   0,   0,   0, 150,   7, 
     16,   0,   1,   0,   0,   0, 
     54,   0,   0,   6, 130,   0, 
     16,   0,   4,   0,   0,   0, 
     58,   0,  16, 128,  65,   0, 
      0,   0,   0,   0,   0,   0, 
     48,   0,   0,   1,  49,   0, 
      0,   7,  66,   0,  16,   0, 
      3,   0,   0,   0,  58,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   4,   0, 
      0,   0,   3,   0,   4,   3, 
     42,   0,  16,   0,   3,   0, 
      0,   0,  54,   0,   0,   5, 
     18,   0,  16,   0,   3,   0, 
      0,   0,  58,   0,  16,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      3,   0,   0,   0,   6,   4, 
     16,   0,   3,   0,   0,   0, 
      6,  20,  16,   0,   0,   0, 
      0,   0,  27,   0,   0,   5, 
     50,   0,  16,   0,   2,   0, 
      0,   0, 230,  10,  16,   0, 
      3,   0,   0,   0,  45,   0, 
      0, 137, 194,   0,   0, 128, 
     67,  85,  21,   0,  18,   0, 
     16,   0,   2,   0,   0,   0, 
     70,  14,  16,   0,   2,   0, 
      0,   0,  70, 126,  16,   0, 
      0,   0,   0,   0,  15,   0, 
      0,   7,  34,   0,  16,   0, 
      2,   0,   0,   0,  70,   0, 
     16,   0,   3,   0,   0,   0, 
     70,   0,  16,   0,   3,   0, 
      0,   0,  75,   0,   0,   5, 
     34,   0,  16,   0,   2,   0, 
      0,   0,  26,   0,  16,   0, 
      2,   0,   0,   0,  56,   0, 
      0,   7,  34,   0,  16,   0, 
      2,   0,   0,   0,  42,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   2,   0, 
      0,   0,  56,   0,   0,   7, 
     34,   0,  16,   0,   2,   0, 
      0,   0,  26,   0,  16,   0, 
      2,   0,   0,   0,  26,   0, 
     16,   0,   2,   0,   0,   0, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   2,   0,   0,   0, 
     26,   0,  16,   0,   2,   0, 
      0,   0,   1,  64,   0,   0, 
     59, 170, 184, 191,  25,   0, 
      0,   5,  34,   0,  16,   0, 
      2,   0,   0,   0,  26,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   8,  18,   0, 
     16,   0,   3,   0,   0,   0, 
     10,   0,  16, 128,  65,   0, 
      0,   0,   0,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  56,   0,   0,   7, 
     18,   0,  16,   0,   3,   0, 
      0,   0,  10,   0,  16,   0, 
      3,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 176,  64, 
     56,   0,   0,   7,  18,   0, 
     16,   0,   3,   0,   0,   0, 
     10,   0,  16,   0,   3,   0, 
      0,   0,  10,   0,  16,   0, 
      3,   0,   0,   0,  56,   0, 
      0,   7,  18,   0,  16,   0, 
      3,   0,   0,   0,  10,   0, 
     16,   0,   3,   0,   0,   0, 
      1,  64,   0,   0,  59, 170, 
    184, 191,  25,   0,   0,   5, 
     18,   0,  16,   0,   3,   0, 
      0,   0,  10,   0,  16,   0, 
      3,   0,   0,   0,  52,   0, 
      0,   9,  66,   0,  16,   0, 
      3,   0,   0,   0,  26,   0, 
     16, 128, 129,   0,   0,   0, 
      3,   0,   0,   0,  58,   0, 
     16, 128, 129,   0,   0,   0, 
      4,   0,   0,   0,  29,   0, 
      0,   7,  66,   0,  16,   0, 
      3,   0,   0,   0,  42,   0, 
     16,   0,   3,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,   1,   0,   0,   7, 
     66,   0,  16,   0,   3,   0, 
      0,   0,  42,   0,  16,   0, 
      3,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     50,   0,   0,  10,  66,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16, 128,  65,   0, 
      0,   0,   3,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   7,  18,   0,  16,   0, 
      2,   0,   0,   0,  26,   0, 
     16,   0,   2,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  56,   0,   0,   7, 
     18,   0,  16,   0,   2,   0, 
      0,   0,  10,   0,  16,   0, 
      3,   0,   0,   0,  10,   0, 
     16,   0,   2,   0,   0,   0, 
     50,   0,   0,   9,  18,   0, 
     16,   0,   4,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  42,   0,  16,   0, 
      3,   0,   0,   0,  10,   0, 
     16,   0,   4,   0,   0,   0, 
     56,   0,   0,   7,  18,   0, 
     16,   0,   2,   0,   0,   0, 
     26,   0,  16,   0,   2,   0, 
      0,   0,  10,   0,  16,   0, 
      3,   0,   0,   0,  50,   0, 
      0,   9,  34,   0,  16,   0, 
      4,   0,   0,   0,  10,   0, 
     16,   0,   2,   0,   0,   0, 
     42,   0,  16,   0,   3,   0, 
      0,   0,  26,   0,  16,   0, 
      4,   0,   0,   0,  50,   0, 
      0,   9,  66,   0,  16,   0, 
      4,   0,   0,   0,  10,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   3,   0, 
      0,   0,  42,   0,  16,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   7, 130,   0,  16,   0, 
      4,   0,   0,   0,  58,   0, 
     16,   0,   4,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,  22,   0,   0,   1, 
     54,   0,   0,   5, 226,   0, 
     16,   0,   1,   0,   0,   0, 
      6,   9,  16,   0,   4,   0, 
      0,   0,   0,   0,   0,   7, 
     34,   0,  16,   0,   3,   0, 
      0,   0,  26,   0,  16,   0, 
      3,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     22,   0,   0,   1,  49,   0, 
      0,   7,  66,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,  14,   0,   0,   7, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      1,   0,   0,   0,  42,   0, 
     16,   0,   1,   0,   0,   0, 
     55,   0,   0,   9,  66,   0, 
     16,   0,   0,   0,   0,   0, 
     42,   0,  16,   0,   0,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  26,   0, 
     16,   0,   1,   0,   0,   0, 
     50,   0,   0,   9,  34,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0,   0,  64,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      0,   0,   0,   0,  14,   0, 
      0,   7,  34,   0,  16,   0, 
      0,   0,   0,   0,  58,   0, 
     16,   0,   1,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,   0,   0,   0,   8, 
     66,   0,  16,   0,   0,   0, 
      0,   0,  10,   0,  16, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      0,   0,   0,   0,  50,   0, 
      0,   9,  18,  32,  16,   0, 
      0,   0,   0,   0,  26,   0, 
     16,   0,   0,   0,   0,   0, 
     42,   0,  16,   0,   0,   0, 
      0,   0,  10,   0,  16,   0, 
      0,   0,   0,   0,  62,   0, 
      0,   1,  83,  84,  65,  84, 
    148,   0,   0,   0,  59,   0, 
      0,   0,   5,   0,   0,   0, 
      0,   0,   0,   0,   2,   0, 
      0,   0,  38,   0,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,   1,   0,   0,   0, 
      2,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   2,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   8,   0,   0,   0, 
      1,   0,   0,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0
};
