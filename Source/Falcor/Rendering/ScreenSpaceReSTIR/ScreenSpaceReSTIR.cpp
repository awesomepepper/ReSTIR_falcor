/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "ScreenSpaceReSTIR.h"
#include "Utils/Color/ColorHelpers.slang"
#include "Utils/Sampling/AliasTable.h"
#include "Utils/Debug/PixelDebug.h"
#include "Utils/Scripting/ScriptBindings.h"
#include <random>
#include <tuple>
#include "Params.slang"

namespace Falcor
{
namespace
{
const char kReflectTypesFile[] = "Rendering/ScreenSpaceReSTIR/ReflectTypes.cs.slang";
const char kUpdateEmissiveTriangles[] = "Rendering/ScreenSpaceReSTIR/UpdateEmissiveTriangles.cs.slang";
const char kGenerateLightTilesFile[] = "Rendering/ScreenSpaceReSTIR/GenerateLightTiles.cs.slang";
const char kInitialResamplingFile[] = "Rendering/ScreenSpaceReSTIR/InitialResampling.cs.slang";
const char kTemporalResamplingFile[] = "Rendering/ScreenSpaceReSTIR/TemporalResampling.cs.slang";
const char kSpatialResamplingFile[] = "Rendering/ScreenSpaceReSTIR/SpatialResampling.cs.slang";
const char kEvaluateFinalSamplesFile[] = "Rendering/ScreenSpaceReSTIR/EvaluateFinalSamples.cs.slang";
const char kGIResamplingFile[] = "Rendering/ScreenSpaceReSTIR/GIResampling.cs.slang";
const char kGIClearReservoirsFile[] = "Rendering/ScreenSpaceReSTIR/GIClearReservoirs.cs.slang";

//const std::string kShaderModel = "6_5";
const std::string kShaderModel = "SM6_5";

const std::string kSurfaceData = "surfaceData";
const std::string kNormalDepth = "normalDepth";
const std::string kFinalSamples = "finalSamples";
const std::string kInitialSamples = "initialSamples";
const std::string kPrevReservoirs = "prevReservoirs";
const std::string kReservoirs = "reservoirs";
const std::string kGIReservoirCount = "giReservoirCount";

const std::string kNeighborOffsets = "neighborOffsets";
const std::string kFrameDim = "frameDim";
const std::string kFrameIndex = "frameIndex";
const std::string kPrevSurfaceData = "prevSurfaceData";
const std::string kMotionVectors = "motionVectors";
const std::string kPrevNormalDepth = "prevNormalDepth";
const std::string kTemporalMaxSamples = "temporalMaxSamples";
const std::string kSpatialMaxSamples = "spatialMaxSamples";
const std::string kReservoirCount = "reservoirCount";
const std::string kMaxSampleAge = "maxSampleAge";
const std::string kCameraOrigin = "cameraOrigin";
const std::string kPrevCameraOrigin = "prevCameraOrigin";
const std::string kViewProj = "viewProj";
const std::string kPrevViewProj = "prevViewProj";
const std::string kForceClearReservoirs = "forceClearReservoirs";
const std::string kNormalThreshold = "normalThreshold";
const std::string kDepthThreshold = "depthThreshold";
const std::string kEmissiveTriangles = "emissiveTriangles";
const std::string kEmissiveTriangleCount = "emissiveTriangleCount";
const std::string kLights = "lights";
const std::string kBrdfCutoff = "brdfCutoff";
const std::string kNeighborCount = "neighborCount";
const std::string kGatherRadius = "gatherRadius";
const std::string kLightTileData = "lightTileData";
const std::string kLightTileCount = "lightTileCount";
const std::string kLightTileSize = "lightTileSize";

const std::string kDebugOutput = "debugOutput";
const std::string kFrameCount = "frameCount";
const std::string kReservoirBuffer0 = "reservoirBuffer0";
const std::string kReservoirBuffer1 = "reservoirBuffer1";

const std::string kSceneVar = "gScene";

const std::string kEnvLightSelectionProbability = "envLightSelectionProbability";
const std::string kEmissiveLightSelectionProbability = "emissiveLightSelectionProbability";
const std::string kAnalyticLightSelectionProbability = "analyticLightSelectionProbability";

const std::string kEnvLightLuminance = "envLightLuminance";
// const std::string kEmissiveTriangles = "emissiveTriangles";
const std::string kEnvLightAliasTable = "envLightAliasTable";
const std::string kEmissiveLightAliasTable = "emissiveLightAliasTable";
const std::string kAnalyticLightAliasTable = "analyticLightAliasTable";
const std::string kEnvLightLuminanceFactor = "envLightLuminanceFactor";

const Gui::DropdownList kDebugOutputList = {
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::Disabled, "Disabled"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::Position, "Position"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::Depth, "Depth"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::Normal, "Normal"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::FaceNormal, "FaceNormal"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::DiffuseWeight, "DiffuseWeight"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::SpecularWeight, "SpecularWeight"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::SpecularRoughness, "SpecularRoughness"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::PackedNormal, "PackedNormal"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::PackedDepth, "PackedDepth"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::InitialWeight, "InitialWeight"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::TemporalReuse, "TemporalReuse"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::SpatialReuse, "SpatialReuse"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::FinalSampleDir, "FinalSampleDir"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::FinalSampleDistance, "FinalSampleDistance"},
    {(uint32_t)ScreenSpaceReSTIR::DebugOutput::FinalSampleLi, "FinalSampleLi"},
};

const Gui::DropdownList kReSTIRModeList = {
    {(uint32_t)ScreenSpaceReSTIR::ReSTIRMode::InputOnly, "Input Only"},
    {(uint32_t)ScreenSpaceReSTIR::ReSTIRMode::TemporalOnly, "Temporal Reuse Only"},
    {(uint32_t)ScreenSpaceReSTIR::ReSTIRMode::TemporalAndBiasedSpatial, "Temporal + Biased Spatial Reuse"},
    {(uint32_t)ScreenSpaceReSTIR::ReSTIRMode::TemporalAndUnbiasedSpatial, "Temporal + Unbiased Spatial Reuse"},
};

const Gui::DropdownList kTargetPdfList = {
    {(uint32_t)ScreenSpaceReSTIR::TargetPDF::IncomingRadiance, "Incoming Radiance"},
    {(uint32_t)ScreenSpaceReSTIR::TargetPDF::OutgoingRadiance, "Outgoing Radiance"},
};

const Gui::DropdownList kSpatialReusePatternList = {
    {(uint32_t)SpatialReusePattern::Default, std::string("Default")},
};

const uint32_t kNeighborOffsetCount = 8192;
} // namespace

ScreenSpaceReSTIR::SharedPtr ScreenSpaceReSTIR::create(
    ref<Device> pDevice,
    const ref<Scene>& pScene,
    const Options::SharedPtr& options,
    int numReSTIRInstances,
    int ReSTIRInstanceID
)
{
    return SharedPtr(new ScreenSpaceReSTIR(pDevice, pScene, options, numReSTIRInstances, ReSTIRInstanceID));
}

ScreenSpaceReSTIR::ScreenSpaceReSTIR(
    ref<Device> pDevice,
    const ref<Scene>& pScene,
    const Options::SharedPtr& options,
    int numReSTIRInstances,
    int ReSTIRInstanceID
)
    : mOptions(options), mpScene(pScene), mpDevice(pDevice)
{
    assert(mpScene);

    mpPixelDebug = make_ref<PixelDebug>(pDevice);

    // Create compute pass for reflecting data types.
    ProgramDesc desc;
    DefineList defines;
    defines.add(mpScene->getSceneDefines());
    defines.add(getLightsDefines());
    desc.addShaderLibrary(kReflectTypesFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
    mpReflectTypes = ComputePass::create(pDevice, desc, defines);

    // Create neighbor offset texture.
    mpNeighborOffsets = createNeighborOffsetTexture(kNeighborOffsetCount);

    mNumReSTIRInstances = numReSTIRInstances;
    mReSTIRInstanceIndex = ReSTIRInstanceID;
    mFrameIndex = mReSTIRInstanceIndex;
}

DefineList ScreenSpaceReSTIR::getDefines() const
{
    DefineList defines;
    defines.add("SCREEN_SPACE_RESTIR_USE_DI", mOptions->useReSTIRDI ? "1" : "0");
    defines.add("SCREEN_SPACE_RESTIR_USE_GI", mOptions->useReSTIRGI ? "1" : "0");
    defines.add("SCREEN_SPACE_RESTIR_GI_DIFFUSE_THRESHOLD", std::to_string(mOptions->diffuseThreshold));
    defines.add("RESTIR_GI_USE_RESTIR_N", mOptions->reSTIRGIUseReSTIRN ? "1" : "0");
    return defines;
}

void ScreenSpaceReSTIR::setShaderData(const ShaderVar& var) const
{
    var[kSurfaceData] = mpSurfaceData;
    var[kNormalDepth] = mpNormalDepthTexture;
    var[kFinalSamples] = mpFinalSamples;

    var[kFrameDim] = mFrameDim;

    // ReSTIR GI.
    var[kInitialSamples] = mpGIInitialSamples;
    // var[kPrevReservoirs] = mpGIReservoirs[(mFrameIndex + 0) % 2];
    // var[kReservoirs] = mpGIReservoirs[(mFrameIndex + 1) % 2];
    var[kPrevReservoirs] = mpGIReservoirs[((mFrameIndex - mReSTIRInstanceIndex) / mNumReSTIRInstances + 0) % 2];
    var[kReservoirs] = mpGIReservoirs[((mFrameIndex - mReSTIRInstanceIndex) / mNumReSTIRInstances + 1) % 2];

    var[kGIReservoirCount] = mOptions->reSTIRGIReservoirCount;
}

void ScreenSpaceReSTIR::enablePass(bool enabled)
{
    mOptions->enabled = enabled;
}

bool ScreenSpaceReSTIR::renderUI(Gui::Widgets& widget)
{
    if (mReSTIRInstanceIndex != 0)
        return false;

    bool dirty = false;

    dirty = widget.checkbox("Enable Pass", mOptions->enabled);

    mRecompile |= widget.checkbox("Use ReSTIR DI", mOptions->useReSTIRDI);
    widget.tooltip("Enable ReSTIR for direct illumination.");

    mRecompile |= widget.checkbox("Use ReSTIR GI", mOptions->useReSTIRGI);
    widget.tooltip("Enable ReSTIR for indirect illumination.");

    if (auto group = widget.group("Debugging"))
    {
        mRecompile |= group.dropdown("Debug output", kDebugOutputList, reinterpret_cast<uint32_t&>(mOptions->debugOutput));
        mpPixelDebug->renderUI(group);
    }

    if (auto group = widget.group("Common options"))
    {
        dirty |= widget.var("Normal threshold", mOptions->normalThreshold, 0.f, 1.f);
        widget.tooltip("Normal cosine threshold for reusing temporal samples or spatial neighbor samples.");

        dirty |= widget.var("Depth threshold", mOptions->depthThreshold, 0.f, 1.f);
        widget.tooltip("Relative depth threshold for reusing temporal samples or spatial neighbor samples.");
    }

    if (auto groupDI = widget.group("ReSTIR DI options"))
    {
        if (auto group = widget.group("Light selection weights"))
        {
            mRecompile |= group.var("Environment", mOptions->envLightWeight, 0.f, 1.f);
            group.tooltip("Relative weight for selecting the env map when sampling a light.");

            mRecompile |= group.var("Emissive", mOptions->emissiveLightWeight, 0.f, 1.f);
            group.tooltip("Relative weight for selecting an emissive light when sampling a light.");

            mRecompile |= group.var("Analytic", mOptions->analyticLightWeight, 0.f, 1.f);
            group.tooltip("Relative weight for selecting an analytical light when sampling a light.");
        }

        if (auto group = widget.group("Emissive lights"))
        {
            mRecompile |= group.checkbox("Use emissive texture for sampling", mOptions->useEmissiveTextureForSampling);
            group.tooltip("Use emissive texture for light sample evaluation.");

            mRecompile |= group.checkbox("Use emissive texture for shading", mOptions->useEmissiveTextureForShading);
            group.tooltip("Use emissive texture for shading.");

            mRecompile |= group.checkbox("Use local emissive triangles", mOptions->useLocalEmissiveTriangles);
            group.tooltip("Use local emissive triangle data structure (for more efficient sampling/evaluation).");
        }

        if (auto group = widget.group("Light tiles"))
        {
            mRecompile |= group.var("Tile count", mOptions->lightTileCount, 1u, 1024u);
            group.tooltip("Number of light tiles to compute.");

            mRecompile |= group.var("Tile size", mOptions->lightTileSize, 1u, 8192u);
            group.tooltip("Number of lights per light tile.");
        }

        if (auto group = widget.group("Visibility", true))
        {
            mRecompile |= group.checkbox("Use alpha test", mOptions->useAlphaTest);
            group.tooltip("Use alpha testing on non-opaque triangles.");

            mRecompile |= group.checkbox("Use initial visibility", mOptions->useInitialVisibility);
            group.tooltip("Check visibility on inital sample.");

            mRecompile |= group.checkbox("Use final visibility", mOptions->useFinalVisibility);
            group.tooltip("Check visibility on final sample.");

            if (mOptions->useFinalVisibility)
            {
                mRecompile |= group.checkbox("Reuse final visibility", mOptions->reuseFinalVisibility);
                group.tooltip("Reuse final visibility temporally.");
            }
        }

        if (auto group = widget.group("Initial resampling", true))
        {
            mRecompile |= group.var("Screen tile size", mOptions->screenTileSize, 1u, 128u);
            group.tooltip("Size of screen tile that samples from the same light tile.");

            mRecompile |= group.var("Initial light sample count", mOptions->initialLightSampleCount, 1u, 1024u);
            group.tooltip("Number of initial light samples to resample per pixel.");

            mRecompile |= group.var("Initial BRDF sample count", mOptions->initialBRDFSampleCount, 0u, 16u);
            group.tooltip("Number of initial BRDF samples to resample per pixel.");

            dirty |= group.var("BRDF Cutoff", mOptions->brdfCutoff, 0.f, 1.f);
            group.tooltip("Value in range [0,1] to determine how much to shorten BRDF rays.");
        }

        if (auto group = widget.group("Temporal resampling", true))
        {
            dirty |= group.checkbox("Use temporal resampling", mOptions->useTemporalResampling);

            mRecompile |= group.var("Max history length", mOptions->maxHistoryLength, 0u, 100u);
            group.tooltip("Maximum temporal history length.");
        }

        if (auto group = widget.group("Spatial resampling", true))
        {
            dirty |= group.checkbox("Use spatial resampling", mOptions->useSpatialResampling);

            dirty |= group.var("Iterations", mOptions->spatialIterations, 0u, 8u);
            group.tooltip("Number of spatial resampling iterations.");

            dirty |= group.var("Neighbor count", mOptions->spatialNeighborCount, 0u, 32u);
            group.tooltip("Number of neighbor samples to resample per pixel and iteration.");

            dirty |= group.var("Gather radius", mOptions->spatialGatherRadius, 5u, 40u);
            group.tooltip("Radius to gather samples from.");
        }

        mRecompile |= widget.checkbox("Use pairwise MIS", mOptions->usePairwiseMIS);
        widget.tooltip("Use pairwise MIS when combining samples.");

        mRecompile |= widget.checkbox("Unbiased", mOptions->unbiased);
        widget.tooltip("Use unbiased version of ReSTIR by querying extra visibility rays.");
    }

    if (auto group = widget.group("ReSTIR GI options"))
    {
        mRecompile |= group.dropdown("Mode", kReSTIRModeList, reinterpret_cast<uint32_t&>(mOptions->reSTIRMode));
        mRecompile |= group.dropdown("Target PDF", kTargetPdfList, reinterpret_cast<uint32_t&>(mOptions->targetPdf));

        dirty |= group.var("Max Temporal Samples", mOptions->reSTIRGITemporalMaxSamples, 0u, 1000u);
        group.tooltip("Maximum number of temporal samples.");

        dirty |= group.var("Max Spatial Samples", mOptions->reSTIRGISpatialMaxSamples, 0u, 5000u);
        group.tooltip("Maximum number of temporal samples.");

        dirty |= group.var("Reservoir Count", mOptions->reSTIRGIReservoirCount, 1u, 32u);
        group.tooltip("Number of reservoirs per pixel.");

        dirty |= group.checkbox("use ReSTIR N", mOptions->reSTIRGIUseReSTIRN);
        group.tooltip("Try to execute ReSTIR GI N times (with a new initial samples for each time).");

        dirty |= group.var("Max Sample Age", mOptions->reSTIRGIMaxSampleAge, 3u, 1000u);
        group.tooltip("Maximum frames that a sample can survive.");

        dirty |= group.checkbox("Enable Spatial Weight Clamping", mOptions->reSTIRGIEnableSpatialWeightClamping);
        dirty |= group.var("Spatial Weight Clamp Threshold", mOptions->reSTIRGISpatialWeightClampThreshold, 1.f, 1000.f);

        dirty |= group.checkbox("Enable Jacobian Clamping", mOptions->reSTIRGIEnableJacobianClamping);
        dirty |= group.var("Jacobian Clamp Threshold", mOptions->reSTIRGIJacobianClampTreshold, 1.f, 1000.f);

        dirty |= group.checkbox("Enable Temporal Jacobian", mOptions->reSTIREnableTemporalJacobian);

        mRecompile |= group.var("Diffuse Threshold", mOptions->diffuseThreshold, 0.f, 1.f);
        group.tooltip("Do not use ReSTIR GI on pixels whose diffuse component lower than this value.");

        dirty |= group.checkbox("Force Clear Reservoirs", mOptions->forceClearReservoirs);
        group.tooltip("Force clear reservoirs.");
    }

    mRecompile |= mRequestReallocate;
    dirty |= mRecompile;

    if (mRequestReallocate)
        mRequestParentRecompile = true;

    return dirty;
}

// void ScreenSpaceReSTIR::setOptions(const Options& options)
//{
//     if (std::memcmp(&options, &mOptions, sizeof(Options)) != 0)
//     {
//         mOptions = options;
//         mRecompile = true;
//     }
// }

void ScreenSpaceReSTIR::beginFrame(RenderContext* pRenderContext, const uint2& frameDim)
{
    if (mOptions->enabled)
    {
        mFrameDim = frameDim;

        prepareResources(pRenderContext);

        mpPixelDebug->beginFrame(pRenderContext, mFrameDim);
    }
}

void ScreenSpaceReSTIR::endFrame(RenderContext* pRenderContext)
{
    if (mOptions->enabled)
    {
        // mFrameIndex++;
        mFrameIndex += mNumReSTIRInstances;

        // Swap surface data.
        std::swap(mpSurfaceData, mpPrevSurfaceData);

        // Swap reservoirs.
        std::swap(mpReservoirs, mpPrevReservoirs);

        mpPixelDebug->endFrame(pRenderContext);
    }
}

void ScreenSpaceReSTIR::updateReSTIRDI(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors)
{
    FALCOR_PROFILER(pRenderContext, "ScreenSpaceReSTIR::updateReSTIRDI");
    // PROFILER("ScreenSpaceReSTIR::updateReSTIRDI");

    if (mOptions->enabled)
    {
        mTotalRISPasses = 2 /*tile generation*/ + 1 /*initial resampling*/ + 1 /*temporal*/ + mOptions->spatialIterations /*spatial*/;
        mCurRISPass = 0;
        prepareLighting(pRenderContext);
        updatePrograms();

        if (mOptions->useReSTIRGI)
        {
            reSTIRGIClearPass(pRenderContext);
        }

        if (!mOptions->useReSTIRDI)
            return;
        updateEmissiveTriangles(pRenderContext);
        generateLightTiles(pRenderContext);
        initialResampling(pRenderContext);
        temporalResampling(pRenderContext, pMotionVectors);
        spatialResampling(pRenderContext);
        evaluateFinalSamples(pRenderContext);
    }
    else
    {
        mRecompile = false;
    }
}

void ScreenSpaceReSTIR::updateReSTIRGI(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors)
{
    if (!mOptions->useReSTIRGI)
        return;

    FALCOR_PROFILER(pRenderContext, "ScreenSpaceReSTIR::updateReSTIRGI");
    if (mOptions->enabled)
    {
        auto rootVar = mpGIResampling->getRootVar();
        mpPixelDebug->prepareProgram(mpGIResampling->getProgram(), rootVar);

        mpScene->bindShaderDataForRaytracing(pRenderContext, rootVar, 0);

        auto var = rootVar["CB"]["gGIResampling"];
        var[kNeighborOffsets] = mpNeighborOffsets;
        var[kFrameDim] = mFrameDim;
        var[kFrameIndex] = mFrameIndex;
        var[kSurfaceData] = mpSurfaceData;
        var[kPrevSurfaceData] = mpPrevSurfaceData;
        var[kMotionVectors] = pMotionVectors;
        var[kNormalDepth] = mpNormalDepthTexture;
        var[kPrevNormalDepth] = mpPrevNormalDepthTexture;
        var[kTemporalMaxSamples] = mOptions->reSTIRGITemporalMaxSamples;
        var[kSpatialMaxSamples] = mOptions->reSTIRGISpatialMaxSamples;
        var[kReservoirCount] = mOptions->reSTIRGIReservoirCount;
        var[kMaxSampleAge] = mOptions->reSTIRGIMaxSampleAge;
        var[kCameraOrigin] = mpScene->getCamera()->getPosition();
        var[kPrevCameraOrigin] = mPrevCameraOrigin;
        var[kViewProj] = mpScene->getCamera()->getViewProjMatrixNoJitter();
        var[kPrevViewProj] = mPrevViewProj;
        var[kForceClearReservoirs] = mOptions->forceClearReservoirs;
        var[kNormalThreshold] = mOptions->normalThreshold;
        var[kDepthThreshold] = mOptions->depthThreshold;
        var[kInitialSamples] = mpGIInitialSamples;
        // var[kSpatialWeightClampThreshold] = mOptions->reSTIRGISpatialWeightClampThreshold;
        // var[kEnableSpatialWeightClamping] = mOptions->reSTIRGIEnableSpatialWeightClamping;
        // var[kJacobianClampThreshold] = mOptions->reSTIRGIJacobianClampTreshold;
        // var[kEnableJacobianClamping] = mOptions->reSTIRGIEnableJacobianClamping;
        // var[kEnableTemporalJacobian] = mOptions->reSTIREnableTemporalJacobian;

        // var[kPrevReservoirs] = mpGIReservoirs[(mFrameIndex + 0) % 2];
        // var[kReservoirs] = mpGIReservoirs[(mFrameIndex + 1) % 2];
        var[kPrevReservoirs] = mpGIReservoirs[((mFrameIndex - mReSTIRInstanceIndex) / mNumReSTIRInstances + 0) % 2];
        var[kReservoirs] = mpGIReservoirs[((mFrameIndex - mReSTIRInstanceIndex) / mNumReSTIRInstances + 1) % 2];

        mpGIResampling->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1);

        pRenderContext->blit(mpNormalDepthTexture->getSRV(), mpPrevNormalDepthTexture->getRTV());

        mPrevCameraOrigin = mpScene->getCamera()->getPosition();
        mPrevViewProj = mpScene->getCamera()->getViewProjMatrixNoJitter();
    }
}

void ScreenSpaceReSTIR::prepareResources(RenderContext* pRenderContext)
{
    // Create light tile buffers.
    {
        uint32_t elementCount = mOptions->lightTileCount * mOptions->lightTileSize;
        if (!mpLightTileData || mpLightTileData->getElementCount() < elementCount)
        {
            mpLightTileData = mpDevice->createStructuredBuffer(
                mpReflectTypes["lightTileData"],
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }
    }

    // Create screen sized buffers.
    {
        uint32_t elementCount = mFrameDim.x * mFrameDim.y;
        if (!mpSurfaceData || mpSurfaceData->getElementCount() < elementCount)
        {
            mpSurfaceData = mpDevice->createStructuredBuffer(
                mpReflectTypes["surfaceData"],
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }
        if (!mpPrevSurfaceData || mpPrevSurfaceData->getElementCount() < elementCount)
        {
            mpPrevSurfaceData = mpDevice->createStructuredBuffer(
                mpReflectTypes["surfaceData"],
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }
        if (!mpReservoirs || mpReservoirs->getElementCount() < elementCount || mRequestReallocate)
        {
            // mpReservoirs = mpDevice->createStructuredBuffer(mpReflectTypes["reservoirs"], elementCount, ResourceBindFlags::ShaderResource
            // | ResourceBindFlags::UnorderedAccess, MemoryType::DeviceLocal, nullptr, false);
            //  WARNING: this assumes we use the uint4 packedReservoir by default (Reservoir.slang)
            mpReservoirs = mpDevice->createStructuredBuffer(
                16,
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }
        if (!mpPrevReservoirs || mpPrevReservoirs->getElementCount() < elementCount || mRequestReallocate)
        {
            // mpPrevReservoirs = mpDevice->createStructuredBuffer(mpReflectTypes["reservoirs"], elementCount,
            // ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, MemoryType::DeviceLocal, nullptr, false);
            //  WARNING: this assumes we use the uint4 packedReservoir by default (Reservoir.slang)
            mpPrevReservoirs = mpDevice->createStructuredBuffer(
                16,
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }

        if (!mpFinalSamples || mpFinalSamples->getElementCount() < elementCount || mRequestReallocate)
        {
            // mpFinalSamples = mpDevice->createStructuredBuffer(mpReflectTypes["finalSamples"], elementCount,
            // ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, MemoryType::DeviceLocal, nullptr, false);
            mpFinalSamples = mpDevice->createStructuredBuffer(
                32,
                elementCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }

        mRequestReallocate = false;

        int initialReservoirCount = mOptions->reSTIRGIUseReSTIRN ? elementCount * mOptions->reSTIRGIReservoirCount : elementCount;

        if (!mpGIInitialSamples || initialReservoirCount != mpGIInitialSamples->getElementCount())
        {
            mpGIInitialSamples = mpDevice->createStructuredBuffer(
                mpReflectTypes["giReservoirs"],
                initialReservoirCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
            if (mpGIInitialSamples->getStructSize() % 16 != 0)
                logWarning("PackedGIReservoir struct size is not a multiple of 16B");
        }

        int reservoirCount = elementCount * 2 * mOptions->reSTIRGIReservoirCount;

        for (int i = 0; i < 2; i++)
        {
            if (!mpGIReservoirs[i] || mpGIReservoirs[i]->getElementCount() != reservoirCount)
            {
                mpGIReservoirs[i] = mpDevice->createStructuredBuffer(
                    mpReflectTypes["giReservoirs"],
                    reservoirCount,
                    ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                    MemoryType::DeviceLocal,
                    nullptr,
                    false
                );
                if (mpGIReservoirs[i]->getStructSize() % 16 != 0)
                    logWarning("PackedGIReservoir struct size is not a multiple of 16B");
            }
        }
    }

    // Create normal/depth texture.
    if (!mpNormalDepthTexture || mpNormalDepthTexture->getWidth() != mFrameDim.x || mpNormalDepthTexture->getHeight() != mFrameDim.y)
    {
        mpNormalDepthTexture = mpDevice->createTexture2D(
            mFrameDim.x,
            mFrameDim.y,
            ResourceFormat::R32Uint,
            1,
            1,
            nullptr,
            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess
        );
    }

    if (!mpPrevNormalDepthTexture || mpPrevNormalDepthTexture->getWidth() != mFrameDim.x ||
        mpPrevNormalDepthTexture->getHeight() != mFrameDim.y)
    {
        mpPrevNormalDepthTexture = mpDevice->createTexture2D(
            mFrameDim.x,
            mFrameDim.y,
            ResourceFormat::R32Uint,
            1,
            1,
            nullptr,
            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::RenderTarget
        );
    }

    // Create debug texture.
    if (!mpDebugOutputTexture || mpDebugOutputTexture->getWidth() != mFrameDim.x || mpDebugOutputTexture->getHeight() != mFrameDim.y)
    {
        mpDebugOutputTexture = mpDevice->createTexture2D(
            mFrameDim.x,
            mFrameDim.y,
            ResourceFormat::RGBA32Float,
            1,
            1,
            nullptr,
            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess
        );
    }
}

void ScreenSpaceReSTIR::prepareLighting(RenderContext* pRenderContext)
{
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RenderSettingsChanged))
        mRecompile = true;

    // Setup alias table for env light.
    if (mpScene->useEnvLight())
    {
        const auto& envMap = mpScene->getEnvMap();
        if (!mpEnvLightLuminance || !mpEnvLightAliasTable)
        {
            const auto& texture = mpScene->getEnvMap()->getEnvMap();
            auto luminances = computeEnvLightLuminance(pRenderContext, texture);
            mpEnvLightLuminance = mpDevice->createTypedBuffer<float>(
                (uint32_t)luminances.size(), ResourceBindFlags::ShaderResource, MemoryType::DeviceLocal, luminances.data()
            );
            mpEnvLightAliasTable = buildEnvLightAliasTable(texture->getWidth(), texture->getHeight(), luminances, mRng);
            mRecompile = true;
        }

        mEnvLightLuminanceFactor = luminance(envMap->getIntensity() * envMap->getTint());
    }
    else
    {
        if (mpEnvLightLuminance)
        {
            mpEnvLightLuminance = nullptr;
            mpEnvLightAliasTable = nullptr;
            mRecompile = true;
        }
    }

    // Setup alias table for emissive lights.
    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        if (!mpEmissiveLightAliasTable)
        {
            auto lightCollection = mpScene->getLightCollection(pRenderContext);
            lightCollection->update(pRenderContext);
            if (lightCollection->getActiveLightCount(pRenderContext) > 0)
            {
                mpEmissiveTriangles = mpDevice->createStructuredBuffer(
                    mpReflectTypes["emissiveTriangles"],
                    lightCollection->getTotalLightCount(),
                    ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                    MemoryType::DeviceLocal,
                    nullptr,
                    false
                );
                mpEmissiveLightAliasTable = buildEmissiveLightAliasTable(pRenderContext, lightCollection, mRng);
                mRecompile = true;
            }
        }
    }
    else
    {
        if (mpEmissiveTriangles)
        {
            mpEmissiveTriangles = nullptr;
            mpEmissiveLightAliasTable = nullptr;
            mRecompile = true;
        }
    }

    // Setup alias table for analytic lights.
    if (mpScene->useAnalyticLights())
    {
        if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCountChanged))
            mpAnalyticLightAliasTable = nullptr;
        if (!mpAnalyticLightAliasTable)
        {
            std::vector<ref<Light>> lights;
            for (uint32_t i = 0; i < mpScene->getLightCount(); ++i)
            {
                const auto& light = mpScene->getLight(i);
                if (light->isActive())
                    lights.push_back(mpScene->getLight(i));
            }
            if (!lights.empty())
            {
                mpAnalyticLightAliasTable = buildAnalyticLightAliasTable(pRenderContext, lights, mRng);
                mRecompile = true;
            }
        }
    }
    else
    {
        if (mpAnalyticLightAliasTable)
        {
            mpAnalyticLightAliasTable = nullptr;
            mRecompile = true;
        }
    }

    // Compute light selection probabilities.
    auto& probs = mLightSelectionProbabilities;
    probs.envLight = mpEnvLightAliasTable ? mOptions->envLightWeight : 0.f;
    probs.emissiveLights = mpEmissiveLightAliasTable ? mOptions->emissiveLightWeight : 0.f;
    probs.analyticLights = mpAnalyticLightAliasTable ? mOptions->analyticLightWeight : 0.f;
    float total = probs.envLight + probs.emissiveLights + probs.analyticLights;
    if (total > 0.f)
    {
        probs.envLight /= total;
        probs.emissiveLights /= total;
        probs.analyticLights /= total;
    }
}

void ScreenSpaceReSTIR::updatePrograms()
{
    if (!mRecompile)
        return;

    DefineList commonDefines;

    commonDefines.add(getDefines());
    commonDefines.add(mpScene->getSceneDefines());
    commonDefines.add(getLightsDefines());
    commonDefines.add("USE_ALPHA_TEST", mOptions->useAlphaTest ? "1" : "0");
    commonDefines.add("DEBUG_OUTPUT", std::to_string((uint32_t)mOptions->debugOutput));

    // UpdateEmissiveTriangles

    {
        DefineList defines = commonDefines;

        if (!mpUpdateEmissiveTriangles)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kUpdateEmissiveTriangles).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpUpdateEmissiveTriangles = ComputePass::create(mpDevice, desc, defines, false);
        }

        //mpScene->bindShaderData(mpUpdateEmissiveTriangles->getRootVar()[kSceneVar]);
        mpScene->bindShaderData(mpUpdateEmissiveTriangles->getRootVar()["CB"][kSceneVar]);
        mpUpdateEmissiveTriangles->getProgram()->addDefines(defines);
        mpUpdateEmissiveTriangles->setVars(nullptr);
    }

    // GenerateLightTiles

    {
        DefineList defines = commonDefines;

        defines.add("LIGHT_TILE_COUNT", std::to_string(mOptions->lightTileCount));
        defines.add("LIGHT_TILE_SIZE", std::to_string(mOptions->lightTileSize));

        auto [envLightSampleCount, emissiveLightSampleCount, analyticLightSampleCount] =
            mLightSelectionProbabilities.getSampleCount(mOptions->lightTileSize);
        defines.add("ENV_LIGHT_SAMPLE_COUNT", std::to_string(envLightSampleCount));
        defines.add("EMISSIVE_LIGHT_SAMPLE_COUNT", std::to_string(emissiveLightSampleCount));
        defines.add("ANALYTIC_LIGHT_SAMPLE_COUNT", std::to_string(analyticLightSampleCount));

        if (!mpGenerateLightTiles)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kGenerateLightTilesFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpGenerateLightTiles = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpGenerateLightTiles->getRootVar()[kSceneVar]);
        mpGenerateLightTiles->getProgram()->addDefines(defines);
        mpGenerateLightTiles->setVars(nullptr);
    }

    // InitialResampling

    {
        DefineList defines = commonDefines;

        defines.add("LIGHT_TILE_COUNT", std::to_string(mOptions->lightTileCount));
        defines.add("LIGHT_TILE_SIZE", std::to_string(mOptions->lightTileSize));

        defines.add("SCREEN_TILE_SIZE", std::to_string(mOptions->screenTileSize));
        defines.add("INITIAL_LIGHT_SAMPLE_COUNT", std::to_string(mOptions->initialLightSampleCount));
        defines.add("INITIAL_BRDF_SAMPLE_COUNT", std::to_string(mOptions->initialBRDFSampleCount));

        // Only need to check visibility if either temporal or spatial reuse is active.
        bool checkVisibility = mOptions->useInitialVisibility & (mOptions->useTemporalResampling || mOptions->useSpatialResampling);
        defines.add("CHECK_VISIBILITY", checkVisibility ? "1" : "0");

        if (!mpInitialResampling)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kInitialResamplingFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpInitialResampling = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpInitialResampling->getRootVar()[kSceneVar]);
        mpInitialResampling->getProgram()->addDefines(defines);
        mpInitialResampling->setVars(nullptr);
    }

    // TemporalResampling

    {
        DefineList defines = commonDefines;

        defines.add("MAX_HISTORY_LENGTH", std::to_string(mOptions->maxHistoryLength));
        // TODO: We currently disable pairwise MIS in the temporal resampling pass.
        // It seems to lead to a lot of variance under camera movement.
        defines.add("USE_PAIRWISE_MIS", "0");
        // TODO: We currently skip shadow rays in the temporal resampling pass.
        // This is not always correct, need to figure out when it needs to be enabled.
        // defines.add("UNBIASED", mOptions->unbiased ? "1" : "0");
        defines.add("UNBIASED", "0");

        if (!mpTemporalResampling)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kTemporalResamplingFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpTemporalResampling = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpTemporalResampling->getRootVar()[kSceneVar]);
        mpTemporalResampling->getProgram()->addDefines(defines);
        mpTemporalResampling->setVars(nullptr);
    }

    // SpatialResampling

    {
        DefineList defines = commonDefines;

        defines.add("NEIGHBOR_OFFSET_COUNT", std::to_string(mpNeighborOffsets->getWidth()));
        defines.add("USE_PAIRWISE_MIS", mOptions->usePairwiseMIS ? "1" : "0");

        defines.add("UNBIASED", mOptions->unbiased ? "1" : "0");

        if (!mpSpatialResampling)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kSpatialResamplingFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpSpatialResampling = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpSpatialResampling->getRootVar()[kSceneVar]);
        mpSpatialResampling->getProgram()->addDefines(defines);
        mpSpatialResampling->setVars(nullptr);
    }

    // EvaluateFinalSamples

    {
        DefineList defines = commonDefines;

        defines.add("USE_VISIBILITY", mOptions->useFinalVisibility ? "1" : "0");
        defines.add("REUSE_VISIBILITY", (mOptions->useFinalVisibility && mOptions->reuseFinalVisibility) ? "1" : "0");

        if (!mpEvaluateFinalSamples)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kEvaluateFinalSamplesFile)
                .csEntry("main")
                .setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpEvaluateFinalSamples = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpEvaluateFinalSamples->getRootVar()[kSceneVar]);
        mpEvaluateFinalSamples->getProgram()->addDefines(defines);
        mpEvaluateFinalSamples->setVars(nullptr);
    }

    {
        DefineList defines = commonDefines;

        defines.add("RESTIR_MODE", std::to_string((int)mOptions->reSTIRMode));
        defines.add("RESTIR_TARGET_FUNCTION", std::to_string((int)mOptions->targetPdf));
        defines.add("NEIGHBOR_OFFSET_COUNT", std::to_string(mpNeighborOffsets->getWidth()));

        if (!mpGIClearReservoirs)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kGIClearReservoirsFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpGIClearReservoirs = ComputePass::create(mpDevice, desc, defines, false);
        }
        mpGIClearReservoirs->getProgram()->addDefines(defines);
        mpGIClearReservoirs->setVars(nullptr);

        if (!mpGIResampling)
        {
            ProgramDesc desc;
            desc.addShaderLibrary(kGIResamplingFile).csEntry("main").setShaderModel(Falcor::stringToEnum<ShaderModel>(kShaderModel));
            mpGIResampling = ComputePass::create(mpDevice, desc, defines, false);
        }

        mpScene->bindShaderData(mpGIResampling->getRootVar()[kSceneVar]);
        mpGIResampling->getProgram()->addDefines(defines);
        mpGIResampling->setVars(nullptr);
    }

    mRecompile = false;
    mResetTemporalReservoirs = true;
}

void ScreenSpaceReSTIR::updateEmissiveTriangles(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "updateEmissiveTriangles");

    if (!mOptions->useLocalEmissiveTriangles || !mpEmissiveTriangles)
        return;

    mpScene->bindShaderData(mpUpdateEmissiveTriangles->getRootVar()[kSceneVar]);

    // auto var = mpUpdateEmissiveTriangles["CB"]["gUpdateEmissiveTriangles"];
    auto var = mpUpdateEmissiveTriangles->getRootVar()["CB"]["gUpdateEmissiveTriangles"];

    uint32_t triangleCount = mpEmissiveTriangles->getElementCount();

    var[kEmissiveTriangles] = mpEmissiveTriangles;
    var[kEmissiveTriangleCount] = triangleCount;

    mpUpdateEmissiveTriangles->execute(pRenderContext, uint32_t(triangleCount), 1, 1);
}

void ScreenSpaceReSTIR::generateLightTiles(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "generateLightTiles");

    mpScene->bindShaderData(mpGenerateLightTiles->getRootVar()[kSceneVar]);

    // auto var = mpGenerateLightTiles["CB"]["gGenerateLightTiles"];
    auto var = mpGenerateLightTiles->getRootVar()["CB"]["gGenerateLightTiles"];

    var[kLightTileData] = mpLightTileData;
    setLightsShaderData(var[kLights]);
    var[kFrameIndex] = mTotalRISPasses * mFrameIndex + mCurRISPass;
    mCurRISPass += 2;

    mpGenerateLightTiles->execute(pRenderContext, uint32_t(mOptions->lightTileSize), uint32_t(mOptions->lightTileCount), 1);
}

void ScreenSpaceReSTIR::initialResampling(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "initialResampling");

    mpScene->bindShaderData(mpInitialResampling->getRootVar()[kSceneVar]);

    auto rootVar = mpInitialResampling->getRootVar();

    mpScene->bindShaderDataForRaytracing(pRenderContext, rootVar, 0);
    mpPixelDebug->prepareProgram(mpInitialResampling->getProgram(), rootVar);

    auto var = rootVar["CB"]["gInitialResampling"];
    var[kSurfaceData] = mpSurfaceData;
    var[kNormalDepth] = mpNormalDepthTexture;
    var[kLightTileData] = mpLightTileData;
    var[kReservoirs] = mpReservoirs;
    var[kDebugOutput] = mpDebugOutputTexture;
    setLightsShaderData(var[kLights]);
    var[kFrameDim] = mFrameDim;
    var[kFrameIndex] = mTotalRISPasses * mFrameIndex + mCurRISPass;
    var[kBrdfCutoff] = mOptions->brdfCutoff;
    mCurRISPass++;

    mpInitialResampling->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1);
}

void ScreenSpaceReSTIR::temporalResampling(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors)
{
    FALCOR_PROFILER(pRenderContext, "temporalResampling");

    if (mResetTemporalReservoirs)
    {
        mResetTemporalReservoirs = false;
        return;
    }

    if (!mOptions->useTemporalResampling)
        return;

    mpScene->bindShaderData(mpTemporalResampling->getRootVar()[kSceneVar]);

    auto rootVar = mpTemporalResampling->getRootVar();

    mpScene->bindShaderDataForRaytracing(pRenderContext, rootVar, 0);
    mpPixelDebug->prepareProgram(mpTemporalResampling->getProgram(), rootVar);

    auto var = rootVar["CB"]["gTemporalResampling"];
    var[kSurfaceData] = mpSurfaceData;
    var[kPrevSurfaceData] = mpPrevSurfaceData;
    var[kMotionVectors] = pMotionVectors;
    var[kReservoirs] = mpReservoirs;
    var[kPrevReservoirs] = mpPrevReservoirs;
    var[kDebugOutput] = mpDebugOutputTexture;
    setLightsShaderData(var[kLights]);
    var[kFrameDim] = mFrameDim;
    var[kFrameIndex] = mTotalRISPasses * mFrameIndex + mCurRISPass;
    var[kNormalThreshold] = mOptions->normalThreshold;
    var[kDepthThreshold] = mOptions->depthThreshold;
    mCurRISPass++;

    mpTemporalResampling->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1);
}

void ScreenSpaceReSTIR::spatialResampling(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "spatialResampling");

    if (!mOptions->useSpatialResampling)
        return;

    mpScene->bindShaderData(mpSpatialResampling->getRootVar()[kSceneVar]);

    auto rootVar = mpSpatialResampling->getRootVar();

    mpScene->bindShaderDataForRaytracing(pRenderContext, rootVar, 0);
    mpPixelDebug->prepareProgram(mpSpatialResampling->getProgram(), rootVar);

    auto var = rootVar["CB"]["gSpatialResampling"];
    var[kSurfaceData] = mpSurfaceData;
    var[kNormalDepth] = mpNormalDepthTexture;
    var[kDebugOutput] = mpDebugOutputTexture;
    var[kNeighborOffsets] = mpNeighborOffsets;
    setLightsShaderData(var[kLights]);
    var[kFrameDim] = mFrameDim;
    var[kNormalThreshold] = mOptions->normalThreshold;
    var[kDepthThreshold] = mOptions->depthThreshold;
    var[kNeighborCount] = mOptions->spatialNeighborCount;
    var[kGatherRadius] = (float)mOptions->spatialGatherRadius;

    for (uint32_t iteration = 0; iteration < mOptions->spatialIterations; ++iteration)
    {
        std::swap(mpReservoirs, mpPrevReservoirs);
        var[kReservoirs] = mpReservoirs;
        var[kPrevReservoirs] = mpPrevReservoirs;
        var[kFrameIndex] = mTotalRISPasses * mFrameIndex + mCurRISPass;
        mCurRISPass += 1;
        mpSpatialResampling->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1);
    }
}

void ScreenSpaceReSTIR::evaluateFinalSamples(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "evaluateFinalSamples");

    mpScene->bindShaderData(mpEvaluateFinalSamples->getRootVar()[kSceneVar]);

    auto rootVar = mpEvaluateFinalSamples->getRootVar();

    mpScene->bindShaderDataForRaytracing(pRenderContext, rootVar, 0);
    mpPixelDebug->prepareProgram(mpEvaluateFinalSamples->getProgram(), rootVar);

    auto var = rootVar["CB"]["gEvaluateFinalSamples"];
    var[kSurfaceData] = mpSurfaceData;
    var[kReservoirs] = mpReservoirs;
    var[kFinalSamples] = mpFinalSamples;
    var[kDebugOutput] = mpDebugOutputTexture;
    setLightsShaderData(var[kLights]);
    var[kFrameDim] = mFrameDim;

    mpEvaluateFinalSamples->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1);
}

void ScreenSpaceReSTIR::reSTIRGIClearPass(RenderContext* pRenderContext)
{
    FALCOR_PROFILER(pRenderContext, "reSTIRGIClearPass");

    // Bind resources.
    auto var = mpGIClearReservoirs->getRootVar()["CB"]["gGIClearReservoirs"];

    var[kFrameDim] = mFrameDim;
    var[kFrameCount] = mFrameIndex;
    var[kForceClearReservoirs] = mOptions->forceClearReservoirs;
    var[kReservoirCount] = mOptions->reSTIRGIReservoirCount;

    var[kInitialSamples] = mpGIInitialSamples;
    var[kReservoirBuffer0] = mpGIReservoirs[0];
    var[kReservoirBuffer1] = mpGIReservoirs[1];

    mpGIClearReservoirs->execute(pRenderContext, mFrameDim.x, mFrameDim.y, 1u);
}

DefineList ScreenSpaceReSTIR::getLightsDefines() const
{
    DefineList defines;

    uint32_t envIndexBits = 26;
    uint32_t envPositionBits = 4;

    uint32_t emissiveIndexBits = 22;
    uint32_t emissivePositionBits = 8;

    uint32_t analyticIndexBits = 14;
    uint32_t analyticPositionBits = 16;

    auto computeIndexPositionBits = [](const ref<AliasTable>& aliasTable, uint32_t& indexBits, uint32_t& positionBits)
    {
        if (!aliasTable)
            return;
        uint32_t count = aliasTable->getCount();
        indexBits = 0;
        while (count > 0)
        {
            ++indexBits;
            count >>= 1;
        }
        if (indexBits & 1)
            ++indexBits;
        if (indexBits >= 30)
            throw std::exception("Count too large to be represented in 30 bits");
        positionBits = 30 - indexBits;
    };

    computeIndexPositionBits(mpEnvLightAliasTable, envIndexBits, envPositionBits);
    computeIndexPositionBits(mpEmissiveLightAliasTable, emissiveIndexBits, emissivePositionBits);
    computeIndexPositionBits(mpAnalyticLightAliasTable, analyticIndexBits, analyticPositionBits);

    defines.add("USE_ENV_LIGHT", mpScene->useEnvLight() ? "1" : "0");
    defines.add("USE_EMISSIVE_LIGHTS", mpScene->useEmissiveLights() ? "1" : "0");
    defines.add("USE_ANALYTIC_LIGHTS", mpScene->useAnalyticLights() ? "1" : "0");

    defines.add("LIGHT_SAMPLE_ENV_INDEX_BITS", std::to_string(envIndexBits));
    defines.add("LIGHT_SAMPLE_ENV_POSITION_BITS", std::to_string(envPositionBits));
    defines.add("LIGHT_SAMPLE_EMISSIVE_INDEX_BITS", std::to_string(emissiveIndexBits));
    defines.add("LIGHT_SAMPLE_EMISSIVE_POSITION_BITS", std::to_string(emissivePositionBits));
    defines.add("LIGHT_SAMPLE_ANALYTIC_INDEX_BITS", std::to_string(analyticIndexBits));
    defines.add("LIGHT_SAMPLE_ANALYTIC_POSITION_BITS", std::to_string(analyticPositionBits));

    defines.add("USE_EMISSIVE_TEXTURE_FOR_SAMPLING", mOptions->useEmissiveTextureForSampling ? "1" : "0");
    defines.add("USE_EMISSIVE_TEXTURE_FOR_SHADING", mOptions->useEmissiveTextureForShading ? "1" : "0");
    defines.add("USE_LOCAL_EMISSIVE_TRIANGLES", mOptions->useLocalEmissiveTriangles ? "1" : "0");

    return defines;
}

void ScreenSpaceReSTIR::setLightsShaderData(const ShaderVar& var) const
{
    var[kEnvLightLuminance] = mpEnvLightLuminance;
    var[kEmissiveTriangles] = mpEmissiveTriangles;

    if (mpEnvLightAliasTable)
        mpEnvLightAliasTable->bindShaderData(var[kEnvLightAliasTable]);
    if (mpEmissiveLightAliasTable)
        mpEmissiveLightAliasTable->bindShaderData(var[kEmissiveLightAliasTable]);
    if (mpAnalyticLightAliasTable)
        mpAnalyticLightAliasTable->bindShaderData(var[kAnalyticLightAliasTable]);

    var[kEnvLightLuminanceFactor] = mEnvLightLuminanceFactor;

    var[kEnvLightSelectionProbability] = mLightSelectionProbabilities.envLight;
    var[kEmissiveLightSelectionProbability] = mLightSelectionProbabilities.emissiveLights;
    var[kAnalyticLightSelectionProbability] = mLightSelectionProbabilities.analyticLights;
}

std::vector<float> ScreenSpaceReSTIR::computeEnvLightLuminance(RenderContext* pRenderContext, const ref<Texture>& texture)
{
    assert(texture);

    uint32_t width = texture->getWidth();
    uint32_t height = texture->getHeight();

    // Read texel data from the env map texture so we can create an alias table of samples proportional to intensity.
    std::vector<uint8_t> texelsRaw;
    if (getFormatType(texture->getFormat()) == FormatType::Float)
    {
        texelsRaw = pRenderContext->readTextureSubresource(texture.get(), 0);
    }
    else
    {
        auto floatTexture = mpDevice->createTexture2D(
            width, height, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::RenderTarget | ResourceBindFlags::ShaderResource
        );
        pRenderContext->blit(texture->getSRV(), floatTexture->getRTV());
        texelsRaw = pRenderContext->readTextureSubresource(floatTexture.get(), 0);
    }

    uint32_t texelCount = width * height;
    uint32_t channelCount = getFormatChannelCount(texture->getFormat());
    const float* texels = reinterpret_cast<const float*>(texelsRaw.data());

    std::vector<float> luminances(texelCount);

    if (channelCount == 1)
    {
        for (uint32_t i = 0; i < texelCount; ++i)
        {
            luminances[i] = texels[0];
            texels += channelCount;
        }
    }
    else if (channelCount == 3 || channelCount == 4)
    {
        for (uint32_t i = 0; i < texelCount; ++i)
        {
            luminances[i] = luminance(float3(texels[0], texels[1], texels[2]));
            texels += channelCount;
        }
    }
    else
    {
        throw std::exception("Invalid number of channels in env map");
    }

    return luminances;
}

ref<AliasTable> ScreenSpaceReSTIR::buildEnvLightAliasTable(
    uint32_t width,
    uint32_t height,
    const std::vector<float>& luminances,
    std::mt19937& rng
)
{
    assert(luminances.size() == width * height);

    std::vector<float> weights(width * height);

    // Computes weights as luminance multiplied by the texel's solid angle.
    for (uint32_t i = 0, y = 0; y < height; ++y)
    {
        float theta = (float)M_PI * (y + 0.5f) / height;
        float solidAngle = (2.f * (float)M_PI / width) * ((float)M_PI / height) * std::sin(theta);

        for (uint32_t x = 0; x < width; ++x, ++i)
        {
            weights[i] = luminances[i] * solidAngle;
        }
    }

    return make_ref<AliasTable>(mpDevice, std::move(weights), rng);
}

ref<AliasTable> ScreenSpaceReSTIR::buildEmissiveLightAliasTable(
    RenderContext* pRenderContext,
    const ref<LightCollection>& lightCollection,
    std::mt19937& rng
)
{
    assert(lightCollection);

    lightCollection->update(pRenderContext);

    const auto& triangles = lightCollection->getMeshLightTriangles(pRenderContext);

    std::vector<float> weights(triangles.size());

    for (size_t i = 0; i < weights.size(); ++i)
    {
        weights[i] = luminance(triangles[i].averageRadiance) * triangles[i].area;
    }

    return make_ref<AliasTable>(mpDevice, std::move(weights), rng);
}

ref<AliasTable> ScreenSpaceReSTIR::buildAnalyticLightAliasTable(
    RenderContext* pRenderContext,
    const std::vector<ref<Light>>& lights,
    std::mt19937& rng
)
{
    std::vector<float> weights(lights.size());

    for (size_t i = 0; i < weights.size(); ++i)
    {
        // TODO: Use weight based on light power.
        weights[i] = 1.f;
    }

    return make_ref<AliasTable>(mpDevice, std::move(weights), rng);
}

ref<Texture> ScreenSpaceReSTIR::createNeighborOffsetTexture(uint32_t sampleCount)
{
    std::unique_ptr<int8_t[]> offsets(new int8_t[sampleCount * 2]);
    const int R = 254;
    const float phi2 = 1.f / 1.3247179572447f;
    float u = 0.5f;
    float v = 0.5f;
    for (uint32_t index = 0; index < sampleCount * 2;)
    {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.f)
            u -= 1.f;
        if (v >= 1.f)
            v -= 1.f;

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f)
            continue;

        offsets[index++] = int8_t((u - 0.5f) * R);
        offsets[index++] = int8_t((v - 0.5f) * R);
    }

    return mpDevice->createTexture1D(sampleCount, ResourceFormat::RG8Snorm, 1, 1, offsets.get());
}

void ScreenSpaceReSTIR::scriptBindings(pybind11::module& m)
{
    pybind11::class_<Options, Options::SharedPtr> options(m, "ScreenSpaceReSTIROptions");
    options.def_readwrite("useReSTIRDI", &Options::useReSTIRDI);
    options.def_readwrite("useReSTIRGI", &Options::useReSTIRGI);
    options.def_readwrite("normalThreshold", &Options::normalThreshold);
    options.def_readwrite("depthThreshold", &Options::depthThreshold);
    options.def_readwrite("envLightWeight", &Options::envLightWeight);
    options.def_readwrite("emissiveLightWeight", &Options::emissiveLightWeight);
    options.def_readwrite("analyticLightWeight", &Options::analyticLightWeight);
    options.def_readwrite("useEmissiveTextureForSampling", &Options::useEmissiveTextureForSampling);
    options.def_readwrite("useEmissiveTextureForShading", &Options::useEmissiveTextureForShading);
    options.def_readwrite("useLocalEmissiveTriangles", &Options::useLocalEmissiveTriangles);
    options.def_readwrite("lightTileCount", &Options::lightTileCount);
    options.def_readwrite("lightTileSize", &Options::lightTileSize);
    options.def_readwrite("useAlphaTest", &Options::useAlphaTest);
    options.def_readwrite("useInitialVisibility", &Options::useInitialVisibility);
    options.def_readwrite("useFinalVisibility", &Options::useFinalVisibility);
    options.def_readwrite("reuseFinalVisibility", &Options::reuseFinalVisibility);
    options.def_readwrite("screenTileSize", &Options::screenTileSize);
    options.def_readwrite("initialLightSampleCount", &Options::initialLightSampleCount);
    options.def_readwrite("initialBRDFSampleCount", &Options::initialBRDFSampleCount);
    options.def_readwrite("brdfCutoff", &Options::brdfCutoff);
    options.def_readwrite("useTemporalResampling", &Options::useTemporalResampling);
    options.def_readwrite("maxHistoryLength", &Options::maxHistoryLength);
    options.def_readwrite("useSpatialResampling", &Options::useSpatialResampling);
    options.def_readwrite("spatialIterations", &Options::spatialIterations);
    options.def_readwrite("spatialNeighborCount", &Options::spatialNeighborCount);
    options.def_readwrite("spatialGatherRadius", &Options::spatialGatherRadius);
    options.def_readwrite("usePairwiseMIS", &Options::usePairwiseMIS);
    options.def_readwrite("unbiased", &Options::unbiased);
    options.def_readwrite("debugOutput", &Options::debugOutput);
    options.def_readwrite("enabled", &Options::enabled);
    options.def_readwrite("reSTIRMode", &Options::reSTIRMode);
    options.def_readwrite("targetPdf", &Options::targetPdf);
    options.def_readwrite("reSTIRGITemporalMaxSamples", &Options::reSTIRGITemporalMaxSamples);
    options.def_readwrite("reSTIRGISpatialMaxSamples", &Options::reSTIRGISpatialMaxSamples);
    options.def_readwrite("reSTIRGIReservoirCount", &Options::reSTIRGIReservoirCount);
    options.def_readwrite("reSTIRGIUseReSTIRN", &Options::reSTIRGIUseReSTIRN);
    options.def_readwrite("reSTIRGIMaxSampleAge", &Options::reSTIRGIMaxSampleAge);
    options.def_readwrite("diffuseThreshold", &Options::diffuseThreshold);
    options.def_readwrite("reSTIRGISpatialWeightClampThreshold", &Options::reSTIRGISpatialWeightClampThreshold);
    options.def_readwrite("reSTIRGIEnableSpatialWeightClamping", &Options::reSTIRGIEnableSpatialWeightClamping);
    options.def_readwrite("reSTIRGIJacobianClampTreshold", &Options::reSTIRGIJacobianClampTreshold);
    options.def_readwrite("reSTIRGIEnableJacobianClamping", &Options::reSTIRGIEnableJacobianClamping);
    options.def_readwrite("reSTIREnableTemporalJacobian", &Options::reSTIREnableTemporalJacobian);
    options.def_readwrite("forceClearReservoirs", &Options::forceClearReservoirs);
}
} // namespace Falcor
