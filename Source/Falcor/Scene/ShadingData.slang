/***************************************************************************
 # Copyright (c) 2015-24, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
import Scene.Material.MaterialData;
import Utils.Geometry.GeometryHelpers;
import Utils.Math.BitTricks; // Required for PACK_BITS/EXTRACT_BITS
import Utils.Math.ShadingFrame;
import Rendering.Materials.IBxDF;

// Add MaterialDefines.slangh for kMaterialTypeBits and MaterialType enum
// #include "Scene/Material/MaterialDefines.slangh"
#include "Utils/HostDeviceShared.slangh"



/** This struct holds information needed for shading a hit point.

    This includes:
    - Geometric properties of the surface.
    - View direction.
    - Texture coordinates.
    - Material ID and header.
    - Index of refraction of the surrounding medium.

    Based on a ShadingData struct, the material system can be queried
    for a material instance at the hit using `gScene.materials.getMaterialInstance()`.
    The material instance has interfaces for sampling and evaluation, as well as for
    querying its properties at the hit.
*/

static const uint kMaterialTypeBits = 8;    ///< Maximum number of bits to store MaterialType.

struct ShadingData : IDifferentiable
{
    // Geometry data
    float3  posW;                       ///< Shading hit position in world space.
    float3  V;                          ///< View direction, -incident direction (-ray.dir)
    float3  N;                      ///< Shading normal at shading hit. (Added from old ShadingData)
    float3  T;                      ///< Shading tangent at shading hit. (Added from old ShadingData)
    float3  B;                      ///< Shading bitangent at shading hit. (Added from old ShadingData)
    float2  uv;                         ///< Texture mapping coordinates.
    float   NdotV;                  ///< Unclamped, can be negative. (Added from old ShadingData)
    float   normalCurvature;        ///< Normal curvature. (Added from old ShadingData)

    ShadingFrame frame;                 ///< Smooth interpolated shading frame in world space at the shading point. The normal is *not* automatically flipped for backfacing hits.
    no_diff float3  faceN;              ///< Face normal in world space, always on the front-facing side.
    no_diff float4  tangentW;           ///< Geometric tangent (xyz) and sign (w) in world space. This is used for orthogonalization. Not normalized, but it is guaranteed to be nonzero and sign (w) is +-1.0.
    bool            frontFacing;        ///< True if primitive seen from the front-facing side, i.e., dot(V, faceN) >= 0.0.
    no_diff float   curveRadius;        ///< Curve cross-sectional radius. Valid only for geometry generated from curves.

    // Pre-loaded material properties (Added from old ShadingData)
    float3  diffuse;
    float   opacity;
    float3  specular;
    float   linearRoughness;
    float3  transmission;
    float3  emissive;
    float   metallic;                   ///< It is used for storing the curve sphere radius when it refers to shading curves.
    float   diffuseTransmission;        ///< Blends between diffuse reflection and transmission lobes.
    float   specularTransmission;       ///< Blends between opaque dielectric BRDF and specular transmissive BSDF.
    float   eta;                        ///< Relative index of refraction (incident IoR / transmissive IoR).

    // Material flags (Added from old ShadingData)
    uint    flags;

    // Accessor functions for flags (Added from old ShadingData)
    static const uint kMaterialTypeOffset = 0;
    static const uint kActiveLobesOffset = kMaterialTypeOffset + kMaterialTypeBits;
    static const uint kDoubleSidedOffset = kActiveLobesOffset + 1; // Corrected offset as per IBxDF.slang, kLobeTypeBits should not be used here
    static const uint kThinSurfaceOffset = kDoubleSidedOffset + 1; // Corrected offset

    [mutating] void setMaterialType(MaterialType type) { flags = PACK_BITS(kMaterialTypeBits, kMaterialTypeOffset, flags, (uint)type); }
    MaterialType getMaterialType() { return MaterialType(EXTRACT_BITS(kMaterialTypeBits, kMaterialTypeOffset, flags)); }
    [mutating] void setActiveLobes(uint activeLobes) { flags = PACK_BITS(kLobeTypeBits, kActiveLobesOffset, flags, activeLobes); } // kLobeTypeBits is defined in IBxDF.slang
    uint getActiveLobes() { return EXTRACT_BITS(kLobeTypeBits, kActiveLobesOffset, flags); }

    // Material data
    MaterialHeader mtl;                 ///< Material header data.
    uint    materialID;                 ///< Material ID at shading location.
    float   IoR;                        ///< Index of refraction for the medium on the front-facing side (i.e. "outside" the material at the hit).

    uint materialGradOffset;            ///< Offset to the material gradient in the gradient buffer.
    uint geometryGradOffset;            ///< Offset to the geometry gradient in the gradient buffer.
    uint threadID;                      ///< Thread ID for gradient aggregation with a hash grid.
    
    // Additional accessors from old ShadingData
    [mutating] void setDoubleSided(bool doubleSided) { flags = PACK_BITS(1, kDoubleSidedOffset, flags, doubleSided ? 1 : 0); }
    bool isDoubleSided() { return flags & (1u << kDoubleSidedOffset); }
    [mutating] void setThinSurface(bool thinSurface) { flags = PACK_BITS(1, kThinSurfaceOffset, flags, thinSurface ? 1 : 0); }
    bool isThinSurface() { return flags & (1u << kThinSurfaceOffset); }

    // Renamed from computeNewRayOrigin to computeRayOrigin for consistency with existing util.
    // The original computeRayOrigin helper from GeometryHelpers.slangh is still used.
    float3 computeNewRayOrigin(bool viewside = true) // Kept original name from source for distinction, but uses computeRayOrigin helper
    {
        return computeRayOrigin(posW, (frontFacing == viewside) ? faceN : -faceN);
    }

    /** Transform vector from the local surface frame to world space. (Added from old ShadingData)
        \param[in] v Vector in local space.
        \return Vector in world space.
    */
    float3 fromLocal(float3 v)
    {
        return frame.T * v.x + frame.B * v.y + frame.N * v.z;
    }

    /** Transform vector from world space to the local surface frame. (Added from old ShadingData)
        \param[in] v Vector in world space.
        \return Vector in local space.
    */
    float3 toLocal(float3 v)
    {
        return float3(dot(v, frame.T), dot(v, frame.B), dot(v, frame.N));
    }
    // Utility functions

    /** Computes new ray origin based on the hit point to avoid self-intersection.
        The method is described in Ray Tracing Gems, Chapter 6, "A Fast and Robust
        Method for Avoiding Self-Intersection" by Carsten Wächter and Nikolaus Binder.
        \param[in] viewside True if the origin should be on the view side (reflection) or false otherwise (transmission).
        \return Ray origin of the new ray.
    */
    float3 computeRayOrigin(bool viewside = true)
    {
        return computeRayOrigin(posW, (frontFacing == viewside) ? faceN : -faceN);
    }

    /** Returns the oriented face normal.
        \return Face normal flipped to the same side as the view vector.
    */
    float3 getOrientedFaceNormal()
    {
        return frontFacing ? faceN : -faceN;
    }
};
