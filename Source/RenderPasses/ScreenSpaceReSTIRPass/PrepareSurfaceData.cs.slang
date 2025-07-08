/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
import LoadShadingData;
import Utils.Color.ColorHelpers;
import Rendering.ScreenSpaceReSTIR.ScreenSpaceReSTIR;

/** Prepares ReSTIR surface data.
 */
struct PrepareSurfaceData
{
    Texture2D<PackedHitInfo> vbuffer;

    ScreenSpaceReSTIR screenSpaceReSTIR;

    uint2 frameDim;

    void execute(uint2 pixel)
    {
        if (any(pixel >= frameDim))
            return;

        ShadingData sd;
        if (loadShadingData(pixel, frameDim, gScene.camera, vbuffer, sd))
        {
            float depth = distance(gScene.camera.getPosition(), sd.posW);
            // float diffuseWeight = luminance(sd.diffuse);
            // // float3 diffuseColor = gScene.materials.getMaterialInstance(sd.materialID).getBSDF().getDiffuseReflectance();
            // // float diffuseWeight = luminance(diffuseColor);
            // float specularWeight = luminance(sd.specular);
            // 获取材质实例和BSDF属性，参考RTXDI实现
            let lod = ExplicitLodTextureSampler(0.f); // TODO: 可根据需要调整lod
            // let hints = getMaterialInstanceHints();   // 如果没有定义getMaterialInstanceHints，可去掉此参数
            // let mi = gScene.materials.getMaterialInstance(sd, lod, hints);
            let mi = gScene.materials.getMaterialInstance(sd, lod);
            let bsdfProperties = mi.getProperties(sd);
            float diffuseWeight = luminance(bsdfProperties.diffuseReflectionAlbedo);
            float specularWeight = luminance(bsdfProperties.specularReflectance);
            screenSpaceReSTIR.setSurfaceData(
                // pixel, sd.computeNewRayOrigin(), depth, sd.N, sd.faceN, diffuseWeight, specularWeight, sd.linearRoughness
                pixel,
                sd.computeRayOrigin(),
                depth,
                sd.frame.N,
                sd.faceN,
                diffuseWeight,
                specularWeight,
                bsdfProperties.roughness
            );
        }
        else
        {
            screenSpaceReSTIR.setInvalidSurfaceData(pixel);
        }
    }
};

cbuffer CB
{
    PrepareSurfaceData gPrepareSurfaceData;
};

[numthreads(16, 16, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID) { gPrepareSurfaceData.execute(dispatchThreadId.xy); }
