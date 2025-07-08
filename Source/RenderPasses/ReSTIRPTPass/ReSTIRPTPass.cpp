/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
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
#include "ReSTIRPTPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include <fstream>

namespace
{
const char kDesc[] = "Path tracer using DXR 1.1 TraceRayInline";

const std::string kGeneratePathsFilename = "RenderPasses/ReSTIRPTPass/GeneratePaths.cs.slang";
const std::string kTracePassFilename = "RenderPasses/ReSTIRPTPass/TracePass.cs.slang";
const std::string kReflectTypesFile = "RenderPasses/ReSTIRPTPass/ReflectTypes.cs.slang";
const std::string kSpatialReusePassFile = "RenderPasses/ReSTIRPTPass/SpatialReuse.cs.slang";
const std::string kTemporalReusePassFile = "RenderPasses/ReSTIRPTPass/TemporalReuse.cs.slang";
const std::string kSpatialPathRetraceFile = "RenderPasses/ReSTIRPTPass/SpatialPathRetrace.cs.slang";
const std::string kTemporalPathRetraceFile = "RenderPasses/ReSTIRPTPass/TemporalPathRetrace.cs.slang";
const std::string kComputePathReuseMISWeightsFile = "RenderPasses/ReSTIRPTPass/ComputePathReuseMISWeights.cs.slang";

// Render pass inputs and outputs.
const std::string kInputVBuffer = "vbuffer";
const std::string kInputMotionVectors = "motionVectors";
const std::string kInputDirectLighting = "directLighting";

const Falcor::ChannelList kInputChannels = {
    {kInputVBuffer, "gVBuffer", "Visibility buffer in packed format", false, ResourceFormat::Unknown},
    {kInputMotionVectors, "gMotionVectors", "Motion vector buffer (float format)", true /* optional */, ResourceFormat::RG32Float},
    {kInputDirectLighting, "gDirectLighting", "Sample count buffer (integer format)", true /* optional */, ResourceFormat::RGBA32Float},
};

const std::string kOutputColor = "color";
const std::string kOutputAlbedo = "albedo";
const std::string kOutputSpecularAlbedo = "specularAlbedo";
const std::string kOutputIndirectAlbedo = "indirectAlbedo";
const std::string kOutputNormal = "normal";
const std::string kOutputReflectionPosW = "reflectionPosW";
const std::string kOutputRayCount = "rayCount";
const std::string kOutputPathLength = "pathLength";
const std::string kOutputDebug = "debug";
const std::string kOutputTime = "time";
const std::string kOutputNRDDiffuseRadianceHitDist = "nrdDiffuseRadianceHitDist";
const std::string kOutputNRDSpecularRadianceHitDist = "nrdSpecularRadianceHitDist";
const std::string kOutputNRDResidualRadianceHitDist = "nrdResidualRadianceHitDist";
const std::string kOutputNRDEmission = "nrdEmission";
const std::string kOutputNRDDiffuseReflectance = "nrdDiffuseReflectance";
const std::string kOutputNRDSpecularReflectance = "nrdSpecularReflectance";

const Falcor::ChannelList kOutputChannels = {
    {kOutputColor, "gOutputColor", "Output color (linear)", true /* optional */},
    {kOutputAlbedo, "gOutputAlbedo", "Output albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm},
    {kOutputNormal, "gOutputNormal", "Output normal (linear)", true /* optional */, ResourceFormat::RGBA16Float},
    {kOutputRayCount, "", "Per-pixel ray count", true /* optional */, ResourceFormat::R32Uint},
    {kOutputPathLength, "", "Per-pixel path length", true /* optional */, ResourceFormat::R32Uint},
    {kOutputDebug, "", "Debug output", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputTime, "", "Per-pixel time", true /* optional */, ResourceFormat::R32Uint},
    {kOutputSpecularAlbedo, "gOutputSpecularAlbedo", "Output specular albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm},
    {kOutputIndirectAlbedo, "gOutputIndirectAlbedo", "Output indirect albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm},
    {kOutputReflectionPosW,
     "gOutputReflectionPosW",
     "Output reflection pos (world space)",
     true /* optional */,
     ResourceFormat::RGBA32Float},
    {kOutputNRDDiffuseRadianceHitDist,
     "gOutputNRDDiffuseRadianceHitDist",
     "Output demodulated diffuse color (linear) and hit distance",
     true /* optional */,
     ResourceFormat::RGBA32Float},
    {kOutputNRDSpecularRadianceHitDist,
     "gOutputNRDSpecularRadianceHitDist",
     "Output demodulated specular color (linear) and hit distance",
     true /* optional */,
     ResourceFormat::RGBA32Float},
    {kOutputNRDResidualRadianceHitDist,
     "gOutputNRDResidualRadianceHitDist",
     "Output residual color (linear) and hit distance",
     true /* optional */,
     ResourceFormat::RGBA32Float},
    {kOutputNRDEmission, "gOutputNRDEmission", "Output primary surface emission", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputNRDDiffuseReflectance,
     "gOutputNRDDiffuseReflectance",
     "Output primary surface diffuse reflectance",
     true /* optional */,
     ResourceFormat::RGBA16Float},
    {kOutputNRDSpecularReflectance,
     "gOutputNRDSpecularReflectance",
     "Output primary surface specular reflectance",
     true /* optional */,
     ResourceFormat::RGBA16Float},
};

// UI variables.
const Gui::DropdownList kColorFormatList = {
    {(uint32_t)ColorFormat::RGBA32F, "RGBA32F (128bpp)"},
    {(uint32_t)ColorFormat::LogLuvHDR, "LogLuvHDR (32bpp)"},
};

const Gui::DropdownList kMISHeuristicList = {
    {(uint32_t)MISHeuristic::Balance, "Balance heuristic"},
    {(uint32_t)MISHeuristic::PowerTwo, "Power heuristic (exp=2)"},
    {(uint32_t)MISHeuristic::PowerExp, "Power heuristic"},
};

const Gui::DropdownList kShiftMappingList = {
    {(uint32_t)ShiftMapping::Reconnection, "Reconnection"},
    {(uint32_t)ShiftMapping::RandomReplay, "Random Replay"},
    {(uint32_t)ShiftMapping::Hybrid, "Hybrid"},
};

const Gui::DropdownList kReSTIRMISList = {
    {(uint32_t)ReSTIRMISKind::Constant, "Constant resampling MIS (with balance-heuristic contribution MIS)"},
    {(uint32_t)ReSTIRMISKind::Talbot, "Talbot resampling MIS"},
    {(uint32_t)ReSTIRMISKind::Pairwise, "Pairwise resampling MIS"},
    {(uint32_t)ReSTIRMISKind::ConstantBinary, "Constant resampling MIS (with 1/|Z| contribution MIS)"},
    {(uint32_t)ReSTIRMISKind::ConstantBiased, "Constant resampling MIS (constant contribution MIS, biased)"},
};

const Gui::DropdownList kReSTIRMISList2 = {
    {(uint32_t)ReSTIRMISKind::Constant, "Constant resampling MIS (with balance-heuristic contribution MIS)"},
    {(uint32_t)ReSTIRMISKind::Talbot, "Talbot resampling MIS"},
    {(uint32_t)ReSTIRMISKind::ConstantBinary, "Constant resampling MIS (with 1/|Z| contribution MIS)"},
    {(uint32_t)ReSTIRMISKind::ConstantBiased, "Constant resampling MIS (constant contribution MIS, biased)"},
};

const Gui::DropdownList kPathReusePatternList = {
    {(uint32_t)PathReusePattern::Block, std::string("Block")},
    {(uint32_t)PathReusePattern::NRooks, std::string("N-Rooks")},
    {(uint32_t)PathReusePattern::NRooksShift, std::string("N-Rooks Shift")},
};

const Gui::DropdownList kSpatialReusePatternList = {
    {(uint32_t)SpatialReusePattern::Default, std::string("Default")},
    {(uint32_t)SpatialReusePattern::SmallWindow, std::string("Small Window")},
};

const Gui::DropdownList kEmissiveSamplerList = {
    {(uint32_t)EmissiveLightSamplerType::Uniform, "Uniform"},
    {(uint32_t)EmissiveLightSamplerType::LightBVH, "LightBVH"},
    {(uint32_t)EmissiveLightSamplerType::Power, "Power"},
};

const Gui::DropdownList kLODModeList = {{(uint32_t)TexLODMode::Mip0, "Mip0"}, {(uint32_t)TexLODMode::RayDiffs, "Ray Diffs"}};

const Gui::DropdownList kPathSamplingModeList = {
    {(uint32_t)PathSamplingMode::ReSTIR, "ReSTIR PT"},
    {(uint32_t)PathSamplingMode::PathReuse, "Bekaert-style Path Reuse"},
    {(uint32_t)PathSamplingMode::PathTracing, "Path Tracing"}};

// Scripting options.
const std::string kSamplesPerPixel = "samplesPerPixel";
const std::string kMaxSurfaceBounces = "maxSurfaceBounces";
const std::string kMaxDiffuseBounces = "maxDiffuseBounces";
const std::string kMaxSpecularBounces = "maxSpecularBounces";
const std::string kMaxTransmissionBounces = "maxTransmissionBounces";
const std::string kAdjustShadingNormals = "adjustShadingNormals";
const std::string kLODBias = "lodBias";
const std::string kSampleGenerator = "sampleGenerator";
const std::string kUseBSDFSampling = "useBSDFSampling";
const std::string kUseNEE = "useNEE";
const std::string kUseMIS = "useMIS";
const std::string kUseRussianRoulette = "useRussianRoulette";
const std::string kScreenSpaceReSTIROptions = "screenSpaceReSTIROptions";
const std::string kUseAlphaTest = "useAlphaTest";
const std::string kMaxNestedMaterials = "maxNestedMaterials";
const std::string kUseLightsInDielectricVolumes = "useLightsInDielectricVolumes";
const std::string kLimitTransmission = "limitTransmission";
const std::string kMaxTransmissionReflectionDepth = "maxTransmissionReflectionDepth";
const std::string kMaxTransmissionRefractionDepth = "maxTransmissionRefractionDepth";
const std::string kDisableCaustics = "disableCaustics";
const std::string kSpecularRoughnessThreshold = "specularRoughnessThreshold";
const std::string kDisableDirectIllumination = "disableDirectIllumination";
const std::string kColorFormat = "colorFormat";
const std::string kMISHeuristic = "misHeuristic";
const std::string kMISPowerExponent = "misPowerExponent";
const std::string kFixedSeed = "fixedSeed";
const std::string kEmissiveSampler = "emissiveSampler";
const std::string kLightBVHOptions = "lightBVHOptions";
const std::string kPrimaryLodMode = "primaryLodMode";
const std::string kUseNRDDemodulation = "useNRDDemodulation";

const std::string kSpatialMisKind = "spatialMisKind";
const std::string kTemporalMisKind = "temporalMisKind";
const std::string kShiftStrategy = "shiftStrategy";
const std::string kRejectShiftBasedOnJacobian = "rejectShiftBasedOnJacobian";
const std::string kJacobianRejectionThreshold = "jacobianRejectionThreshold";
const std::string kNearFieldDistance = "nearFieldDistance";
const std::string kLocalStrategyType = "localStrategyType";

const std::string kTemporalHistoryLength = "temporalHistoryLength";
const std::string kUseMaxHistory = "useMaxHistory";
const std::string kSeedOffset = "seedOffset";
const std::string kEnableTemporalReuse = "enableTemporalReuse";
const std::string kEnableSpatialReuse = "enableSpatialReuse";
const std::string kNumSpatialRounds = "numSpatialRounds";
const std::string kPathSamplingMode = "pathSamplingMode";
const std::string kEnableTemporalReprojection = "enableTemporalReprojection";
const std::string kNoResamplingForTemporalReuse = "noResamplingForTemporalReuse";
const std::string kSpatialNeighborCount = "spatialNeighborCount";
const std::string kFeatureBasedRejection = "featureBasedRejection";
const std::string kSpatialReusePattern = "spatialReusePattern";
const std::string kSmallWindowRestirWindowRadius = "smallWindowRestirWindowRadius";
const std::string kSpatialReuseRadius = "spatialReuseRadius";
const std::string kUseDirectLighting = "useDirectLighting";
const std::string kSeparatePathBSDF = "separatePathBSDF";
const std::string kCandidateSamples = "candidateSamples";
const std::string kTemporalUpdateForDynamicScene = "temporalUpdateForDynamicScene";
const std::string kEnableRayStats = "enableRayStats";

const uint32_t kNeighborOffsetCount = 8192;
} // namespace

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, ReSTIRPTPass>();
}

ReSTIRPTPass::ReSTIRPTPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    if (!mpDevice->isShaderModelSupported(ShaderModel::SM6_5))
        FALCOR_THROW("ReSTIRPTPass requires Shader Model 6.5 support.");
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::RaytracingTier1_1))
        FALCOR_THROW("ReSTIRPTPass requires Raytracing Tier 1.1 support.");

    // Initial default values, moved from old Init()
    mStaticParams = StaticParams();
    mParams = RestirPathTracerParams();
    mEnableTemporalReuse = true;
    mEnableSpatialReuse = true;
    mSpatialReusePattern = SpatialReusePattern::Default;
    mPathReusePattern = PathReusePattern::NRooksShift;
    mSmallWindowRestirWindowRadius = 2;
    mSpatialNeighborCount = 3;
    mSpatialReuseRadius = 20.f;
    mNumSpatialRounds = 1;
    mEnableTemporalReprojection = false;
    mUseMaxHistory = true;
    mUseDirectLighting = true;
    mTemporalHistoryLength = 20;
    mNoResamplingForTemporalReuse = false;
    mSeedOffset = 0; // Ensure seed offset is initialized

    parseProperties(props);
    validateOptions();

    // Create sample generator
    mpSampleGenerator = SampleGenerator::create(mpDevice, mStaticParams.sampleGenerator);

    mpPixelStats = std::make_unique<PixelStats>(mpDevice);
    mpPixelDebug = std::make_unique<PixelDebug>(mpDevice);

    // Initial compile flags and reset
    mRecompile = true;
    mOptionsChanged = true;
    reset(); // Reset accumulators
}

Properties ReSTIRPTPass::getProperties() const
{
    Properties props;
    // Save properties to dictionary here if needed, similar to PathTracer::getProperties
    // For example:
    // props[kSamplesPerPixel] = mStaticParams.samplesPerPixel;
    // Add all your options here for scripting and saving graph
    props[kSamplesPerPixel] = mStaticParams.samplesPerPixel;
    props[kMaxSurfaceBounces] = mStaticParams.maxSurfaceBounces;
    props[kMaxDiffuseBounces] = mStaticParams.maxDiffuseBounces;
    props[kMaxSpecularBounces] = mStaticParams.maxSpecularBounces;
    props[kMaxTransmissionBounces] = mStaticParams.maxTransmissionBounces;
    props[kAdjustShadingNormals] = mStaticParams.adjustShadingNormals;
    props[kSampleGenerator] = (uint32_t)mStaticParams.sampleGenerator;
    props[kUseBSDFSampling] = mStaticParams.useBSDFSampling;
    props[kUseNEE] = mStaticParams.useNEE;
    props[kUseMIS] = mStaticParams.useMIS;
    props[kUseRussianRoulette] = mStaticParams.useRussianRoulette;
    props[kUseAlphaTest] = mStaticParams.useAlphaTest;
    props[kMaxNestedMaterials] = mStaticParams.maxNestedMaterials;
    props[kUseLightsInDielectricVolumes] = mStaticParams.useLightsInDielectricVolumes;
    props[kLimitTransmission] = mStaticParams.limitTransmission;
    props[kMaxTransmissionReflectionDepth] = mStaticParams.maxTransmissionReflectionDepth;
    props[kMaxTransmissionRefractionDepth] = mStaticParams.maxTransmissionRefractionDepth;
    props[kDisableCaustics] = mStaticParams.disableCaustics;
    props[kDisableDirectIllumination] = mStaticParams.disableDirectIllumination;
    props[kColorFormat] = (uint32_t)mStaticParams.colorFormat;
    props[kMISHeuristic] = (uint32_t)mStaticParams.misHeuristic;
    props[kMISPowerExponent] = mStaticParams.misPowerExponent;
    props[kEmissiveSampler] = (uint32_t)mStaticParams.emissiveSampler;
    props[kPrimaryLodMode] = (uint32_t)mStaticParams.primaryLodMode;
    props[kUseNRDDemodulation] = mStaticParams.useNRDDemodulation;
    props[kSpatialMisKind] = (uint32_t)mStaticParams.spatialMisKind;
    props[kTemporalMisKind] = (uint32_t)mStaticParams.temporalMisKind;
    props[kShiftStrategy] = (uint32_t)mStaticParams.shiftStrategy;
    props[kTemporalHistoryLength] = mTemporalHistoryLength;
    props[kUseMaxHistory] = mUseMaxHistory;
    props[kSeedOffset] = mSeedOffset;
    props[kEnableTemporalReuse] = mEnableTemporalReuse;
    props[kEnableSpatialReuse] = mEnableSpatialReuse;
    props[kNumSpatialRounds] = mNumSpatialRounds;
    props[kPathSamplingMode] = (uint32_t)mStaticParams.pathSamplingMode;
    props[kEnableTemporalReprojection] = mEnableTemporalReprojection;
    props[kNoResamplingForTemporalReuse] = mNoResamplingForTemporalReuse;
    props[kSpatialNeighborCount] = mSpatialNeighborCount;
    props[kFeatureBasedRejection] = mFeatureBasedRejection;
    props[kSpatialReusePattern] = (uint32_t)mSpatialReusePattern;
    props[kSmallWindowRestirWindowRadius] = mSmallWindowRestirWindowRadius;
    props[kSpatialReuseRadius] = mSpatialReuseRadius;
    props[kUseDirectLighting] = mUseDirectLighting;
    props[kSeparatePathBSDF] = mStaticParams.separatePathBSDF;
    props[kCandidateSamples] = mStaticParams.candidateSamples;
    props[kTemporalUpdateForDynamicScene] = mStaticParams.temporalUpdateForDynamicScene;
    props[kEnableRayStats] = mEnableRayStats;

    return props;
}

void ReSTIRPTPass::setProperties(const Properties& props)
{
    parseProperties(props);
    validateOptions();

    // Update light sampler options if applicable
    if (mpEmissiveSampler)
    {
        if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
            lightBVHSampler->setOptions(mLightBVHOptions);
    }

    mRecompile = true;
    mOptionsChanged = true;
    reset(); // Reset accumulators when properties change
}

RenderPassReflection ReSTIRPTPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    addInputOutput(reflector, kInputChannels);
    addInputOutput(reflector, kOutputChannels);
    return reflector;
}

void ReSTIRPTPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    if (mRecompile)
    {
        updatePrograms();
        mRecompile = false;
    }

    if (mOptionsChanged)
    {
        // Handle options changed, e.g., update program defines, re-create resources
        mOptionsChanged = false;
    }

    if (!beginFrame(pRenderContext, renderData))
    {
        return;
    }

    generatePaths(pRenderContext, renderData);

    // Implement the rest of the execute logic, e.g., PathReusePass, PathRetracePass, etc.
    // ...

    endFrame(pRenderContext, renderData);
}

void ReSTIRPTPass::renderUI(Gui::Widgets& widget)
{
    // Example UI elements. Add your own based on ReSTIRPTPass options
    bool changed = false;
    if (widget.checkbox("Enable Temporal Reuse", mEnableTemporalReuse))
    {
        changed = true;
    }
    if (widget.checkbox("Enable Spatial Reuse", mEnableSpatialReuse))
    {
        changed = true;
    }

    if (auto s = widget.group("ReSTIRPT Options"))
    {
        changed |= s->var("Samples Per Pixel", mStaticParams.samplesPerPixel, 1u, 128u);
        changed |= s->var("Candidate Samples", mStaticParams.candidateSamples, 1u, 64u);
        changed |= s->var("Max Surface Bounces", mStaticParams.maxSurfaceBounces, 0u, 100u);
        changed |= s->dropdown("Path Sampling Mode", kPathSamplingModeList, (uint32_t&)mStaticParams.pathSamplingMode);
        changed |= s->dropdown("Spatial MIS Kind", kReSTIRMISList, (uint32_t&)mStaticParams.spatialMisKind);
        changed |= s->dropdown("Temporal MIS Kind", kReSTIRMISList2, (uint32_t&)mStaticParams.temporalMisKind);
        changed |= s->dropdown("Shift Strategy", kShiftMappingList, (uint32_t&)mStaticParams.shiftStrategy);
        changed |= s->dropdown("Spatial Reuse Pattern", kSpatialReusePatternList, (uint32_t&)mSpatialReusePattern);
        changed |= s->var("Spatial Reuse Radius", mSpatialReuseRadius, 0.f, 100.f, 0.1f);
        changed |= s->var("Num Spatial Rounds", mNumSpatialRounds, 1u, 10u);
        changed |= s->checkbox("Enable Temporal Reprojection", mEnableTemporalReprojection);
        changed |= s->checkbox("No Resampling For Temporal Reuse", mNoResamplingForTemporalReuse);
        changed |= s->var("Temporal History Length", mTemporalHistoryLength, 1, 100);
        changed |= s->checkbox("Use Max History", mUseMaxHistory);
        changed |= s->checkbox("Use Direct Lighting", mUseDirectLighting);
        changed |= s->checkbox("Feature Based Rejection", mFeatureBasedRejection);
        changed |= s->var("Spatial Neighbor Count", mSpatialNeighborCount, 1, 100);
        changed |= s->var("Small Window ReSTIR Window Radius", mSmallWindowRestirWindowRadius, 1u, 10u);
        changed |= s->checkbox("Temporal Update For Dynamic Scene", mStaticParams.temporalUpdateForDynamicScene);
        changed |= s->checkbox("Enable Ray Stats", mEnableRayStats);
    }

    if (changed)
    {
        mOptionsChanged = true;
        mRecompile = true;
        reset();
    }

    renderRenderingUI(widget);
    renderDebugUI(widget);
    renderStatsUI(widget);
}

void ReSTIRPTPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    if (mpScene)
    {
        // Connect to scene update flags
        // mUpdateFlagsConnection = mpScene->getUpdateFlagsSignal().connect([&](IScene::UpdateFlags flags) { mUpdateFlags |= flags; });
        // Assuming there is no mUpdateFlags and connection in ReSTIRPTPass for now.
    }

    reset(); // Reset accumulators and state when scene changes
    mRecompile = true;
    mOptionsChanged = true;
}

void ReSTIRPTPass::reset()
{
    mParams.frameCount = 0;
    mAccumulatedShadowRayCount = 0;
    mAccumulatedClosestHitRayCount = 0;
    mAccumulatedRayCount = 0;
    mReservoirFrameCount = 0; // Reset internal reservoir frame count
    mpPixelStats->reset();    // Reset pixel stats
    mpPixelDebug->reset();    // Reset pixel debug
}

// Private helper functions implementation

bool ReSTIRPTPass::parseProperties(const Properties& props)
{
    bool changed = false;
    // Parse properties from the 'props' object and set them to mStaticParams and other member variables.
    // This is where you map scripting properties to your internal options.

    // Example for a few properties. Add all your properties here.
    for (const auto& [key, value] : props)
    {
        if (key == kSamplesPerPixel)
        {
            uint32_t newSpp = value;
            if (mStaticParams.samplesPerPixel != newSpp)
            {
                mStaticParams.samplesPerPixel = newSpp;
                changed = true;
            }
        }
        else if (key == kMaxSurfaceBounces)
        {
            uint32_t newBounces = value;
            if (mStaticParams.maxSurfaceBounces != newBounces)
            {
                mStaticParams.maxSurfaceBounces = newBounces;
                changed = true;
            }
        }
        else if (key == kEnableTemporalReuse)
        {
            bool newValue = value;
            if (mEnableTemporalReuse != newValue)
            {
                mEnableTemporalReuse = newValue;
                changed = true;
            }
        }
        // ... add more properties parsing logic for all your options
    }
    return changed;
}

void ReSTIRPTPass::validateOptions()
{
    // Implement validation logic for your options here.
    // E.g., clamping values, checking for invalid combinations.
    if (mParams.specularRoughnessThreshold < 0.f || mParams.specularRoughnessThreshold > 1.f)
    {
        logError("'specularRoughnessThreshold' has invalid value. Clamping to range [0,1].");
        mParams.specularRoughnessThreshold = clamp(mParams.specularRoughnessThreshold, 0.f, 1.f);
    }

    // Static parameters.
    if (mStaticParams.samplesPerPixel < 1 || mStaticParams.samplesPerPixel > kMaxSamplesPerPixel)
    {
        logError("'samplesPerPixel' must be in the range [1, " + std::to_string(kMaxSamplesPerPixel) + "]. Clamping to this range.");
        mStaticParams.samplesPerPixel = std::clamp(mStaticParams.samplesPerPixel, 1u, kMaxSamplesPerPixel);
    }

    auto clampBounces = [](uint32_t& bounces, const std::string& name)
    {
        if (bounces > kMaxBounces)
        {
            logError("'" + name + "' exceeds the maximum supported bounces. Clamping to " + std::to_string(kMaxBounces));
            bounces = kMaxBounces;
        }
    };

    clampBounces(mStaticParams.maxSurfaceBounces, kMaxSurfaceBounces);
    clampBounces(mStaticParams.maxDiffuseBounces, kMaxDiffuseBounces);
    clampBounces(mStaticParams.maxSpecularBounces, kMaxSpecularBounces);
    clampBounces(mStaticParams.maxTransmissionBounces, kMaxTransmissionBounces);

    // Make sure maxSurfaceBounces is at least as many as any of diffuse, specular or transmission.
    uint32_t minSurfaceBounces =
        std::max(mStaticParams.maxDiffuseBounces, std::max(mStaticParams.maxSpecularBounces, mStaticParams.maxTransmissionBounces));
    mStaticParams.maxSurfaceBounces = std::max(mStaticParams.maxSurfaceBounces, minSurfaceBounces);

    if (mStaticParams.maxTransmissionReflectionDepth > mStaticParams.maxTransmissionBounces)
    {
        logWarning(
            "'maxTransmissionReflectionDepth' exceeds `maxTransmissionBounces`. Clamping to " +
            std::to_string(mStaticParams.maxTransmissionBounces)
        );
        mStaticParams.maxTransmissionReflectionDepth = mStaticParams.maxTransmissionBounces;
    }

    if (mStaticParams.maxTransmissionRefractionDepth > mStaticParams.maxTransmissionBounces)
    {
        logWarning(
            "'maxTransmissionRefractionDepth' exceeds `maxTransmissionBounces`. Clamping to " +
            std::to_string(mStaticParams.maxTransmissionBounces)
        );
        mStaticParams.maxTransmissionRefractionDepth = mStaticParams.maxTransmissionBounces;
    }

    if (mStaticParams.primaryLodMode == TexLODMode::RayCones)
    {
        logError("Unsupported tex lod mode. Defaulting to Mip0.");
        mStaticParams.primaryLodMode = TexLODMode::Mip0;
    }
}

void ReSTIRPTPass::updatePrograms()
{
    // This function should recompile shaders or update program defines based on mStaticParams and other options.
    // Similar to PathTracer::updatePrograms()
    DefineList defines = mStaticParams.getDefines(*this);
    // Add other defines based on ReSTIRPTPass specific options
    defines.add("ENABLE_TEMPORAL_REUSE", mEnableTemporalReuse ? "1" : "0");
    defines.add("ENABLE_SPATIAL_REUSE", mEnableSpatialReuse ? "1" : "0");
    // ...

    // Recreate/update your compute passes (mpGeneratePaths, mpTracePass, etc.) with new defines.
    mpGeneratePaths = ComputePass::create(mpDevice, ProgramDesc().addShaderLibrary(kGeneratePathsFilename).csEntry("main"), defines, false);
    // ... similarly for other passes (mpTracePass, mpSpatialReusePass, etc.)
}

void ReSTIRPTPass::prepareResources(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Prepare resources like FBOs, buffers, textures based on current render settings.
    // Similar to PathTracer::prepareResources()
    // For example, allocate/resize mpOutputReservoirs based on output resolution and samples per pixel.
}

void ReSTIRPTPass::setNRDData(const ShaderVar& var, const RenderData& renderData) const
{
    // Set NRD related shader data if applicable
}

void ReSTIRPTPass::preparePathTracer(const RenderData& renderData)
{
    // Prepare path tracer specific data, e.g., update constants buffers
}

void ReSTIRPTPass::resetLighting()
{
    // Reset lighting related data or samplers.
    mpEnvMapSampler = nullptr;
    mpEmissiveSampler = nullptr;
}

void ReSTIRPTPass::prepareMaterials(RenderContext* pRenderContext)
{
    // Prepare materials if needed.
}

bool ReSTIRPTPass::prepareLighting(RenderContext* pRenderContext)
{
    // Prepare lighting, e.g., create light samplers, update light data.
    // Similar to PathTracer::prepareLighting()
    if (!mpScene)
        return false;

    // (Re)create emissive light sampler if scene lights or options have changed
    if (!mpEmissiveSampler || mOptionsChanged) // Also check for scene light changes (e.g. mpScene->getUpdateFlags() &
                                               // IScene::UpdateFlags::SceneLightCollectionChanged)
    {
        switch (mStaticParams.emissiveSampler)
        {
        case EmissiveLightSamplerType::Uniform:
            mpEmissiveSampler = EmissiveUniformSampler::create(mpDevice);
            break;
        case EmissiveLightSamplerType::LightBVH:
            mpEmissiveSampler = LightBVHSampler::create(mpDevice, mpScene);
            if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
                lightBVHSampler->setOptions(mLightBVHOptions);
            break;
        case EmissiveLightSamplerType::Power:
            mpEmissiveSampler = EmissivePowerSampler::create(mpDevice);
            break;
        default:
            FALCOR_UNREACHABLE();
        }
        if (mpEmissiveSampler)
            mpEmissiveSampler->setPRNG(mpSampleGenerator);
        mRecompile = true;
    }

    // (Re)create environment map sampler if needed
    if (mpScene->getEnvMap())
    {
        if (!mpEnvMapSampler || mRecompile) // Or if env map has changed (mpScene->getUpdateFlags() & IScene::UpdateFlags::EnvMapChanged)
        {
            mpEnvMapSampler = EnvMapSampler::create(mpDevice, mpScene->getEnvMap());
        }
    }
    else
    {
        mpEnvMapSampler = nullptr;
    }

    return true;
}

void ReSTIRPTPass::setShaderData(const ShaderVar& var, const RenderData& renderData, bool isPathTracer, bool isPathGenerator) const
{
    // Set common shader data, e.g., scene, camera, sample generator.
    if (!mpScene)
    {
        FALCOR_THROW("Scene is missing");
    }
    mpScene->set;
    var["gScene"] = mpScene->getParameterBlock();
    var["gCamera"] = mpScene->getActiveCamera()->getParameterBlock();
    mpSampleGenerator->set;
    var["gSampleGenerator"] = mpSampleGenerator->getParameterBlock();
    var["gFrameData.frameCount"] = mParams.frameCount;
    var["gFrameData.seed"] = mParams.fixedSeed ? mStaticParams.sampleGenerator : mSeedOffset + mParams.frameCount;

    if (mpEmissiveSampler)
    {
        mpEmissiveSampler->set.var["gEmissiveSampler"] = mpEmissiveSampler->getParameterBlock();
    }
    if (mpEnvMapSampler)
    {
        mpEnvMapSampler->set.var["gEnvMapSampler"] = mpEnvMapSampler->getParameterBlock();
    }

    // Set other common data like frame dimensions, inverse matrices, etc.
}

bool ReSTIRPTPass::renderRenderingUI(Gui::Widgets& widget)
{
    bool changed = false;
    // Add rendering specific UI elements and update changed flag.
    // Example:
    if (auto s = widget.group("Path Tracing Options"))
    {
        // clang-format off
        changed |= s->var("Samples Per Pixel", mStaticParams.samplesPerPixel, 1u, 128u);
        changed |= s->var("Max Surface Bounces", mStaticParams.maxSurfaceBounces, 0u, 100u);
        // ... more rendering options
        // clang-format on
    }
    return changed;
}

bool ReSTIRPTPass::renderDebugUI(Gui::Widgets& widget)
{
    bool changed = false;
    // Add debug specific UI elements.
    if (auto s = widget.group("Debug"))
    {
        // Example:
        // changed |= s->checkbox("Enable Ray Stats", mEnableRayStats);
    }
    return changed;
}

bool ReSTIRPTPass::renderStatsUI(Gui::Widgets& widget)
{
    bool changed = false;
    // Add stats specific UI elements.
    if (auto s = widget.group("Statistics"))
    {
        s->text(fmt::format("Accumulated Rays: {}", mAccumulatedRayCount));
        s->text(fmt::format("Accumulated Shadow Rays: {}", mAccumulatedShadowRayCount));
        s->text(fmt::format("Accumulated Closest Hit Rays: {}", mAccumulatedClosestHitRayCount));
    }
    return changed;
}

bool ReSTIRPTPass::beginFrame(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Handle frame updates, e.g., increment frameCount, update debug data.
    if (!mpScene)
        return false;

    // Update frame count
    mParams.frameCount++;

    // Update scene if needed. PathTracer checks mUpdateFlags.
    // If you have a similar flag in ReSTIRPTPass, check it here.
    // E.g., if (mUpdateFlags != IScene::UpdateFlags::None) {
    //      resetLighting(); // Or other scene update related resets
    //      mRecompile = true;
    //      mOptionsChanged = true;
    //      reset();
    //      mUpdateFlags = IScene::UpdateFlags::None;
    // }

    // Prepare materials and lighting for the frame
    prepareMaterials(pRenderContext);
    if (!prepareLighting(pRenderContext))
    {
        return false;
    }

    // Prepare resources for the current frame
    prepareResources(pRenderContext, renderData);

    return true;
}

void ReSTIRPTPass::endFrame(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Perform end of frame tasks, e.g., read back stats.
    if (mEnableRayStats && mpCountersReadback)
    {
        pRenderContext->readBuffer(mpCounters.get(), mpCountersReadback.get(), 0, mpCounters->getSize());
        mpReadbackFence->cpuSync();
        const uint32_t* pCounters = reinterpret_cast<const uint32_t*>(mpCountersReadback->map());
        mAccumulatedRayCount = pCounters[0];
        mAccumulatedClosestHitRayCount = pCounters[1];
        mAccumulatedShadowRayCount = pCounters[2];
        mpCountersReadback->unmap();
    }
}

void ReSTIRPTPass::generatePaths(RenderContext* pRenderContext, const RenderData& renderData, int sampleId)
{
    // Implement path generation logic.
    // Similar to PathTracer::generatePaths()
    if (!mpGeneratePaths)
        return;

    setShaderData(mpGeneratePaths->get, renderData, false, true);

    mpGeneratePaths->execute(pRenderContext, mpScene->get){};
}

void ReSTIRPTPass::tracePass(
    RenderContext* pRenderContext,
    const RenderData& renderData,
    const ref<ComputePass>& pass,
    const std::string& passName,
    int sampleId
)
{
    // Implement trace pass logic.
    // Similar to PathTracer::tracePass()
    if (!pass)
        return;

    setShaderData(pass->get, renderData, true, false);

    pass->execute(pRenderContext, mpScene->get){};
}

void ReSTIRPTPass::PathReusePass(
    RenderContext* pRenderContext,
    uint32_t restir_i,
    const RenderData& renderData,
    bool temporalReuse,
    int spatialRoundId,
    bool isLastRound
)
{
    // Implement path reuse logic.
}

void ReSTIRPTPass::PathRetracePass(
    RenderContext* pRenderContext,
    uint32_t restir_i,
    const RenderData& renderData,
    bool temporalReuse,
    int spatialRoundId
)
{
    // Implement path retrace logic.
}

ref<Texture> ReSTIRPTPass::createNeighborOffsetTexture(uint32_t sampleCount)
{
    // Create neighbor offset texture for N-Rooks sampling.
    // This is specific to ReSTIRPTPass.
    return nullptr;
}
