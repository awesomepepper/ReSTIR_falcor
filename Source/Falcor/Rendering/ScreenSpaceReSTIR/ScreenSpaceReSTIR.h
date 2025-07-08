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
#include "Utils/Sampling/AliasTable.h"
#include "Utils/Debug/PixelDebug.h"
#include "Utils/Dictionary.h"
#include "Utils/Properties.h"
#include <random>
#include <tuple>
// #include "Params.slang"

namespace Falcor
{
/** Implementation of ReSTIR for direct and global illumination.

    The direct illumination part (ReSTIR DI) is based on
    "Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting"
    by Benedikt Bitterli et al. from 2020.

    The global illumination part (ReSTIR GI) is based on
    "ReSTIR GI: Path Resampling for Real-Time Path Tracing"
    by Yaobin Ouyang et al. from 2021.

    Integrating this module into a renderer requires a few steps:

    - Host:   Call ScreenSpaceReSTIR::beginFrame() on to begin a new frame.
    - Device: Populate surface data (GBuffer) using ScreenSpaceReSTIR::setSurfaceData()/setInvalidSurfaceData().

    For ReSTIR DI:

    - Host:   Call ScreenSpaceReSTIR::updateReSTIRDI() on to run the ReSTIR DI algorithm.
    - Device: Get final light samples using ScreenSpaceReSTIR::getFinalSample() and perform shading.

    For ReSTIR GI:

    - Device: Use a path tracer to generate initial samples, store them using ScreenSpaceReSTIR::setGIInitialSample().
    - Host:   Call ScreenSpaceReSTIR::updateReSTIRGI() to run the ReSTIR GI algorithm.
    - Device: Write a pass to get final samples using ScreenSpaceReSTIR::getGIFinalSample() and perform shading.

    Finally at the end of frame:

    - Host:   Call ScreenSpaceReSTIR::endFrame() to end the frame.

    Also see the ScreenSpaceReSTIRPass render pass for a minimal example on how to use the sampler.
*/
class __declspec(dllexport) ScreenSpaceReSTIR
{
public:
    using SharedPtr = std::shared_ptr<ScreenSpaceReSTIR>;

    /** Enumeration of available debug outputs.
        Note: Keep in sync with definition in Params.slang
    */
    enum class DebugOutput
    {
        Disabled,
        Position,
        Depth,
        Normal,
        FaceNormal,
        DiffuseWeight,
        SpecularWeight,
        SpecularRoughness,
        PackedNormal,
        PackedDepth,
        InitialWeight,
        TemporalReuse,
        SpatialReuse,
        FinalSampleDir,
        FinalSampleDistance,
        FinalSampleLi,
    };

    enum class ReSTIRMode
    {
        InputOnly = 0,
        TemporalOnly = 1,
        TemporalAndBiasedSpatial = 2,
        TemporalAndUnbiasedSpatial = 3
    };

    enum class TargetPDF
    {
        IncomingRadiance = 0,
        OutgoingRadiance = 1
    };

    /** Configuration options.
     */
    struct Options
    {
        using SharedPtr = std::shared_ptr<Options>;
        static SharedPtr create() { return SharedPtr(new Options()); };
        //static SharedPtr create(const Options& other) { return SharedPtr(new Options(other)); };
        static SharedPtr create(const Falcor::Properties& props)
        {
            SharedPtr pOptions = SharedPtr(new Options());
            for (const auto& [key, value] : props)
            {
                if (key == "useReSTIRDI")
                    pOptions->useReSTIRDI = value; // bool
                else if (key == "useReSTIRGI")
                    pOptions->useReSTIRGI = value; // bool
                else if (key == "normalThreshold")
                    pOptions->normalThreshold = value; // float
                else if (key == "depthThreshold")
                    pOptions->depthThreshold = value; // float
                else if (key == "envLightWeight")
                    pOptions->envLightWeight = value; // float
                else if (key == "emissiveLightWeight")
                    pOptions->emissiveLightWeight = value; // float
                else if (key == "analyticLightWeight")
                    pOptions->analyticLightWeight = value; // float
                else if (key == "useEmissiveTextureForSampling")
                    pOptions->useEmissiveTextureForSampling = value; // bool
                else if (key == "useEmissiveTextureForShading")
                    pOptions->useEmissiveTextureForShading = value; // bool
                else if (key == "useLocalEmissiveTriangles")
                    pOptions->useLocalEmissiveTriangles = value; // bool
                else if (key == "lightTileCount")
                    pOptions->lightTileCount = value; // uint32_t
                else if (key == "lightTileSize")
                    pOptions->lightTileSize = value; // uint32_t
                else if (key == "useAlphaTest")
                    pOptions->useAlphaTest = value; // bool
                else if (key == "useInitialVisibility")
                    pOptions->useInitialVisibility = value; // bool
                else if (key == "useFinalVisibility")
                    pOptions->useFinalVisibility = value; // bool
                else if (key == "reuseFinalVisibility")
                    pOptions->reuseFinalVisibility = value; // bool
                else if (key == "screenTileSize")
                    pOptions->screenTileSize = value; // uint32_t
                else if (key == "initialLightSampleCount")
                    pOptions->initialLightSampleCount = value; // uint32_t
                else if (key == "initialBRDFSampleCount")
                    pOptions->initialBRDFSampleCount = value; // uint32_t
                else if (key == "brdfCutoff")
                    pOptions->brdfCutoff = value; // float
                else if (key == "useTemporalResampling")
                    pOptions->useTemporalResampling = value; // bool
                else if (key == "maxHistoryLength")
                    pOptions->maxHistoryLength = value; // uint32_t
                else if (key == "useSpatialResampling")
                    pOptions->useSpatialResampling = value; // bool
                else if (key == "spatialIterations")
                    pOptions->spatialIterations = value; // uint32_t
                else if (key == "spatialNeighborCount")
                    pOptions->spatialNeighborCount = value; // uint32_t
                else if (key == "spatialGatherRadius")
                    pOptions->spatialGatherRadius = value; // uint32_t
                else if (key == "usePairwiseMIS")
                    pOptions->usePairwiseMIS = value; // bool
                else if (key == "unbiased")
                    pOptions->unbiased = value; // bool
                else if (key == "debugOutput")
                    pOptions->debugOutput = (DebugOutput)(int)value; // DebugOutput
                else if (key == "enabled")
                    pOptions->enabled = value; // bool
                else if (key == "reSTIRMode")
                    pOptions->reSTIRMode = (ReSTIRMode)(int)value; // ReSTIRMode
                else if (key == "targetPdf")
                    pOptions->targetPdf = (TargetPDF)(int)value; // TargetPDF
                else if (key == "reSTIRGITemporalMaxSamples")
                    pOptions->reSTIRGITemporalMaxSamples = value; // uint32_t
                else if (key == "reSTIRGISpatialMaxSamples")
                    pOptions->reSTIRGISpatialMaxSamples = value; // uint32_t
                else if (key == "reSTIRGIReservoirCount")
                    pOptions->reSTIRGIReservoirCount = value; // uint32_t
                else if (key == "reSTIRGIUseReSTIRN")
                    pOptions->reSTIRGIUseReSTIRN = value; // bool
                else if (key == "reSTIRGIMaxSampleAge")
                    pOptions->reSTIRGIMaxSampleAge = value; // uint32_t
                else if (key == "diffuseThreshold")
                    pOptions->diffuseThreshold = value; // float
                else if (key == "reSTIRGISpatialWeightClampThreshold")
                    pOptions->reSTIRGISpatialWeightClampThreshold = value; // float
                else if (key == "reSTIRGIEnableSpatialWeightClamping")
                    pOptions->reSTIRGIEnableSpatialWeightClamping = value; // bool
                else if (key == "reSTIRGIJacobianClampTreshold")
                    pOptions->reSTIRGIJacobianClampTreshold = value; // float
                else if (key == "reSTIRGIEnableJacobianClamping")
                    pOptions->reSTIRGIEnableJacobianClamping = value; // bool
                else if (key == "reSTIREnableTemporalJacobian")
                    pOptions->reSTIREnableTemporalJacobian = value; // bool
                else if (key == "forceClearReservoirs")
                    pOptions->forceClearReservoirs = value; // bool
                else
                    logWarning("Unknown field '" + key + "' in ScreenSpaceReSTIR::Options dictionary");
            }
            return pOptions;
        }

        Falcor::Dictionary toDictionary() const
        {
            Falcor::Dictionary dict;
            dict["useReSTIRDI"] = useReSTIRDI;
            dict["useReSTIRGI"] = useReSTIRGI;
            dict["normalThreshold"] = normalThreshold;
            dict["depthThreshold"] = depthThreshold;
            dict["envLightWeight"] = envLightWeight;
            dict["emissiveLightWeight"] = emissiveLightWeight;
            dict["analyticLightWeight"] = analyticLightWeight;
            dict["useEmissiveTextureForSampling"] = useEmissiveTextureForSampling;
            dict["useEmissiveTextureForShading"] = useEmissiveTextureForShading;
            dict["useLocalEmissiveTriangles"] = useLocalEmissiveTriangles;
            dict["lightTileCount"] = lightTileCount;
            dict["lightTileSize"] = lightTileSize;
            dict["useAlphaTest"] = useAlphaTest;
            dict["useInitialVisibility"] = useInitialVisibility;
            dict["useFinalVisibility"] = useFinalVisibility;
            dict["reuseFinalVisibility"] = reuseFinalVisibility;
            dict["screenTileSize"] = screenTileSize;
            dict["initialLightSampleCount"] = initialLightSampleCount;
            dict["initialBRDFSampleCount"] = initialBRDFSampleCount;
            dict["brdfCutoff"] = brdfCutoff;
            dict["useTemporalResampling"] = useTemporalResampling;
            dict["maxHistoryLength"] = maxHistoryLength;
            dict["useSpatialResampling"] = useSpatialResampling;
            dict["spatialIterations"] = spatialIterations;
            dict["spatialNeighborCount"] = spatialNeighborCount;
            dict["spatialGatherRadius"] = spatialGatherRadius;
            dict["usePairwiseMIS"] = usePairwiseMIS;
            dict["unbiased"] = unbiased;
            dict["debugOutput"] = (int)debugOutput;
            dict["enabled"] = enabled;
            dict["reSTIRMode"] = (int)reSTIRMode;
            dict["targetPdf"] = (int)targetPdf;
            dict["reSTIRGITemporalMaxSamples"] = reSTIRGITemporalMaxSamples;
            dict["reSTIRGISpatialMaxSamples"] = reSTIRGISpatialMaxSamples;
            dict["reSTIRGIReservoirCount"] = reSTIRGIReservoirCount;
            dict["reSTIRGIUseReSTIRN"] = reSTIRGIUseReSTIRN;
            dict["reSTIRGIMaxSampleAge"] = reSTIRGIMaxSampleAge;
            dict["diffuseThreshold"] = diffuseThreshold;
            dict["reSTIRGISpatialWeightClampThreshold"] = reSTIRGISpatialWeightClampThreshold;
            dict["reSTIRGIEnableSpatialWeightClamping"] = reSTIRGIEnableSpatialWeightClamping;
            dict["reSTIRGIJacobianClampTreshold"] = reSTIRGIJacobianClampTreshold;
            dict["reSTIRGIEnableJacobianClamping"] = reSTIRGIEnableJacobianClamping;
            dict["reSTIREnableTemporalJacobian"] = reSTIREnableTemporalJacobian;
            dict["forceClearReservoirs"] = forceClearReservoirs;
            return dict;
        }

        Falcor::Properties toProperties() const
        {
            Falcor::Properties props;
            props["useReSTIRDI"] = useReSTIRDI;
            props["useReSTIRGI"] = useReSTIRGI;
            props["normalThreshold"] = normalThreshold;
            props["depthThreshold"] = depthThreshold;
            props["envLightWeight"] = envLightWeight;
            props["emissiveLightWeight"] = emissiveLightWeight;
            props["analyticLightWeight"] = analyticLightWeight;
            props["useEmissiveTextureForSampling"] = useEmissiveTextureForSampling;
            props["useEmissiveTextureForShading"] = useEmissiveTextureForShading;
            props["useLocalEmissiveTriangles"] = useLocalEmissiveTriangles;
            props["lightTileCount"] = lightTileCount;
            props["lightTileSize"] = lightTileSize;
            props["useAlphaTest"] = useAlphaTest;
            props["useInitialVisibility"] = useInitialVisibility;
            props["useFinalVisibility"] = useFinalVisibility;
            props["reuseFinalVisibility"] = reuseFinalVisibility;
            props["screenTileSize"] = screenTileSize;
            props["initialLightSampleCount"] = initialLightSampleCount;
            props["initialBRDFSampleCount"] = initialBRDFSampleCount;
            props["brdfCutoff"] = brdfCutoff;
            props["useTemporalResampling"] = useTemporalResampling;
            props["maxHistoryLength"] = maxHistoryLength;
            props["useSpatialResampling"] = useSpatialResampling;
            props["spatialIterations"] = spatialIterations;
            props["spatialNeighborCount"] = spatialNeighborCount;
            props["spatialGatherRadius"] = spatialGatherRadius;
            props["usePairwiseMIS"] = usePairwiseMIS;
            props["unbiased"] = unbiased;
            props["debugOutput"] = (int)debugOutput;
            props["enabled"] = enabled;
            props["reSTIRMode"] = (int)reSTIRMode;
            props["targetPdf"] = (int)targetPdf;
            props["reSTIRGITemporalMaxSamples"] = reSTIRGITemporalMaxSamples;
            props["reSTIRGISpatialMaxSamples"] = reSTIRGISpatialMaxSamples;
            props["reSTIRGIReservoirCount"] = reSTIRGIReservoirCount;
            props["reSTIRGIUseReSTIRN"] = reSTIRGIUseReSTIRN;
            props["reSTIRGIMaxSampleAge"] = reSTIRGIMaxSampleAge;
            props["diffuseThreshold"] = diffuseThreshold;
            props["reSTIRGISpatialWeightClampThreshold"] = reSTIRGISpatialWeightClampThreshold;
            props["reSTIRGIEnableSpatialWeightClamping"] = reSTIRGIEnableSpatialWeightClamping;
            props["reSTIRGIJacobianClampTreshold"] = reSTIRGIJacobianClampTreshold;
            props["reSTIRGIEnableJacobianClamping"] = reSTIRGIEnableJacobianClamping;
            props["reSTIREnableTemporalJacobian"] = reSTIREnableTemporalJacobian;
            props["forceClearReservoirs"] = forceClearReservoirs;
            return props;
        }

        Options() {}
        Options(const Options& other) { *this = other; }

        // template<typename Archive>
        // void serialize(Archive& ar)
        // {
        //     ar("useReSTIRDI", useReSTIRDI);
        //     ar("useReSTIRGI", useReSTIRGI);
        //     ar("normalThreshold", normalThreshold);
        //     ar("depthThreshold", depthThreshold);
        //     ar("envLightWeight", envLightWeight);
        //     ar("emissiveLightWeight", emissiveLightWeight);
        //     ar("analyticLightWeight", analyticLightWeight);
        //     ar("useEmissiveTextureForSampling", useEmissiveTextureForSampling);
        //     ar("useEmissiveTextureForShading", useEmissiveTextureForShading);
        //     ar("useLocalEmissiveTriangles", useLocalEmissiveTriangles);
        //     ar("lightTileCount", lightTileCount);
        //     ar("lightTileSize", lightTileSize);
        //     ar("useAlphaTest", useAlphaTest);
        //     ar("useInitialVisibility", useInitialVisibility);
        //     ar("useFinalVisibility", useFinalVisibility);
        //     ar("reuseFinalVisibility", reuseFinalVisibility);
        //     ar("screenTileSize", screenTileSize);
        //     ar("initialLightSampleCount", initialLightSampleCount);
        //     ar("initialBRDFSampleCount", initialBRDFSampleCount);
        //     ar("brdfCutoff", brdfCutoff);
        //     ar("useTemporalResampling", useTemporalResampling);
        //     ar("maxHistoryLength", maxHistoryLength);
        //     ar("useSpatialResampling", useSpatialResampling);
        //     ar("spatialIterations", spatialIterations);
        //     ar("spatialNeighborCount", spatialNeighborCount);
        //     ar("spatialGatherRadius", spatialGatherRadius);
        //     ar("usePairwiseMIS", usePairwiseMIS);
        //     ar("unbiased", unbiased);
        //     ar("debugOutput", debugOutput);
        //     ar("enabled", enabled);
        //     ar("reSTIRMode", reSTIRMode);
        //     ar("targetPdf", targetPdf);
        //     ar("reSTIRGITemporalMaxSamples", reSTIRGITemporalMaxSamples);
        //     ar("reSTIRGISpatialMaxSamples", reSTIRGISpatialMaxSamples);
        //     ar("reSTIRGIReservoirCount", reSTIRGIReservoirCount);
        //     ar("reSTIRGIUseReSTIRN", reSTIRGIUseReSTIRN);
        //     ar("reSTIRGIMaxSampleAge", reSTIRGIMaxSampleAge);
        //     ar("diffuseThreshold", diffuseThreshold);
        //     ar("reSTIRGISpatialWeightClampThreshold", reSTIRGISpatialWeightClampThreshold);
        //     ar("reSTIRGIEnableSpatialWeightClamping", reSTIRGIEnableSpatialWeightClamping);
        //     ar("reSTIRGIJacobianClampTreshold", reSTIRGIJacobianClampTreshold);
        //     ar("reSTIRGIEnableJacobianClamping", reSTIRGIEnableJacobianClamping);
        //     ar("reSTIREnableTemporalJacobian", reSTIREnableTemporalJacobian);
        //     ar("forceClearReservoirs", forceClearReservoirs);
        // }

        // Common Options for ReSTIR DI and GI.

        bool useReSTIRDI = true;
        bool useReSTIRGI = false;
        float normalThreshold = 0.5f; ///< Normal cosine threshold for reusing temporal samples or spatial neighbor samples.
        float depthThreshold = 0.1f;  ///< Relative depth threshold for reusing temporal samples or spatial neighbor samples.

        // Options for ReSTIR DI only.

        // Light sampling options.
        float envLightWeight = 1.f;      ///< Relative weight for selecting the env map when sampling a light.
        float emissiveLightWeight = 1.f; ///< Relative weight for selecting an emissive light when sampling a light.
        float analyticLightWeight = 1.f; ///< Relative weight for selecting an analytical light when sampling a light.

        bool useEmissiveTextureForSampling = true; ///< Use emissive texture for light sample evaluation.
        bool useEmissiveTextureForShading = true;  ///< Use emissive texture for shading.
        bool useLocalEmissiveTriangles = false;    ///< Use local emissive triangle data structure (for more efficient sampling/evaluation).

        // Light tile options.
        uint32_t lightTileCount = 128; ///< Number of light tiles to compute.
        uint32_t lightTileSize = 1024; ///< Number of lights per light tile.

        // Visibility options.
        bool useAlphaTest = true;          ///< Use alpha testing on non-opaque triangles.
        bool useInitialVisibility = true;  ///< Check visibility on inital sample.
        bool useFinalVisibility = true;    ///< Check visibility on final sample.
        bool reuseFinalVisibility = false; ///< Reuse final visibility temporally.

        // Initial resampling options.
        uint32_t screenTileSize = 8;           ///< Size of screen tile that samples from the same light tile.
        uint32_t initialLightSampleCount = 32; ///< Number of initial light samples to resample per pixel.
        uint32_t initialBRDFSampleCount = 1;   ///< Number of initial BRDF samples to resample per pixel.
        float brdfCutoff = 0.f;                ///< Value in range [0,1] to determine how much to shorten BRDF rays.

        // Temporal resampling options.
        bool useTemporalResampling = true; ///< Enable temporal resampling.
        uint32_t maxHistoryLength = 20;    ///< Maximum temporal history length.

        // Spatial resampling options.
        bool useSpatialResampling = true;  ///< Enable spatial resampling.
        uint32_t spatialIterations = 1;    ///< Number of spatial resampling iterations.
        uint32_t spatialNeighborCount = 5; ///< Number of neighbor samples to resample per pixel and iteration.
        uint32_t spatialGatherRadius = 30; ///< Radius to gather samples from.

        // General options.
        bool usePairwiseMIS = true; ///< Use pairwise MIS when combining samples.
        bool unbiased = true;       ///< Use unbiased version of ReSTIR by querying extra visibility rays.

        DebugOutput debugOutput = DebugOutput::Disabled;

        bool enabled = true; // (I think) now controls both ReSTIR GI and DI

        // Options for ReSTIR GI only.

        ReSTIRMode reSTIRMode = ReSTIRMode::TemporalAndUnbiasedSpatial; ///< ReSTIR GI Mode.
        TargetPDF targetPdf = TargetPDF::OutgoingRadiance;              ///< Target function mode.
        uint32_t reSTIRGITemporalMaxSamples = 30;                       ///< Maximum M value for temporal reuse stage.
        uint32_t reSTIRGISpatialMaxSamples = 100;                       ///< Maximum M value for spatial reuse stage.
        uint32_t reSTIRGIReservoirCount = 1;                            ///< Number of reservoirs per pixel.
        bool reSTIRGIUseReSTIRN = true;
        uint32_t reSTIRGIMaxSampleAge = 100; ///< Maximum frames that a sample can survive.
        float diffuseThreshold = 0.f; // 0.17f;                     ///< Pixels with diffuse component under this threshold will not use
                                      // ReSTIR GI.
        float reSTIRGISpatialWeightClampThreshold = 10.f;
        bool reSTIRGIEnableSpatialWeightClamping = true;
        float reSTIRGIJacobianClampTreshold = 10.f;
        bool reSTIRGIEnableJacobianClamping = false;
        bool reSTIREnableTemporalJacobian = true;

        bool forceClearReservoirs = false; ///< Force clear temporal and spatial reservoirs.
    };

    /** Create a new instance of the ReSTIR sampler.
        \param[in] pDevice Device.
        \param[in] pScene Scene.
        \param[in] options Configuration options.
    */
    static SharedPtr create(
        ref<Device> pDevice,
        const ref<Scene>& pScene,
        const Options::SharedPtr& options,
        int numReSTIRInstances = 1,
        int ReSTIRInstanceID = 0
    );

    /** Get a list of shader defines for using the ReSTIR sampler.
        \return Returns a list of defines.
    */
    DefineList getDefines() const;

    /** Bind the ReSTIR sampler to a given shader var.
        \param[in] var The shader variable to set the data into.
    */
    void setShaderData(const ShaderVar& var) const;

    /** Render the GUI.
        \return True if options were changed, false otherwise.
    */
    bool renderUI(Gui::Widgets& widget);

    /** Returns the current configuration.
     */
    // const Options& getOptions() const { return mOptions; }

    /** Set the configuration.
     */
    // void setOptions(const Options& options);

    /** Begin a frame.
        Must be called once at the beginning of each frame.
        \param[in] pRenderContext Render context.
        \param[in] frameDim Current frame dimension.
    */
    void beginFrame(RenderContext* pRenderContext, const uint2& frameDim);

    /** End a frame.
        Must be called one at the end of each frame.
        \param[in] pRenderContext Render context.
    */
    void endFrame(RenderContext* pRenderContext);

    /** Update the ReSTIR sampler.
        This runs the ReSTIR DI algorithm and prepares a set of final samples to be queried afterwards.
        Must be called once between beginFrame() and endFrame().
        \param[in] pRenderContext Render context.
        \param[in] pMotionVectors Motion vectors for temporal reprojection.
    */
    void updateReSTIRDI(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors);

    /** Update the ReSTIR sampler.
        This runs the ReSTIR GI algorithm.
        Must be called once between beginFrame() and endFrame().
        \param[in] pRenderContext Render context.
        \param[in] pMotionVectors Motion vectors for temporal reprojection.
    */
    void updateReSTIRGI(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors);

    /** Get the debug output texture.
        \return Returns the debug output texture.
    */
    const ref<Texture>& getDebugOutputTexture() const { return mpDebugOutputTexture; }

    /** Get the pixel debug component.
        \return Returns the pixel debug component.
    */
    const ref<PixelDebug>& getPixelDebug() const { return mpPixelDebug; }

    /** Register script bindings.
     */
    static void scriptBindings(pybind11::module& m);

    void enablePass(bool enabled);

    bool mRequestParentRecompile = true;

    void resetReservoirCount()
    {
        mRecompile = true;
        mFrameIndex = 0;
    };

    void copyRecompileStateFromOtherInstance(ScreenSpaceReSTIR::SharedPtr other)
    {
        mRecompile = other->mRecompile;
        mRequestReallocate = other->mRequestReallocate;
        mResetTemporalReservoirs = other->mResetTemporalReservoirs;
    };
    Options::SharedPtr mOptions; ///< Configuration options.

private:
    ScreenSpaceReSTIR(
        ref<Device> pDevice,
        const ref<Scene>& pScene,
        const Options::SharedPtr& options,
        int numReSTIRInstances = 1,
        int ReSTIRInstanceID = 0
    );

    void prepareResources(RenderContext* pRenderContext);
    void prepareLighting(RenderContext* pRenderContext);

    void updatePrograms();

    void updateEmissiveTriangles(RenderContext* pRenderContext);
    void generateLightTiles(RenderContext* pRenderContext);
    void initialResampling(RenderContext* pRenderContext);
    void temporalResampling(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors);
    void spatialResampling(RenderContext* pRenderContext);
    void evaluateFinalSamples(RenderContext* pRenderContext);

    void reSTIRGIClearPass(RenderContext* pRenderContext);

    DefineList getLightsDefines() const;
    void setLightsShaderData(const ShaderVar& var) const;

    std::vector<float> computeEnvLightLuminance(RenderContext* pRenderContext, const ref<Texture>& texture);
    ref<AliasTable> buildEnvLightAliasTable(uint32_t width, uint32_t height, const std::vector<float>& luminances, std::mt19937& rng);
    ref<AliasTable> buildEmissiveLightAliasTable(
        RenderContext* pRenderContext,
        const ref<LightCollection>& lightCollection,
        std::mt19937& rng
    );
    ref<AliasTable> buildAnalyticLightAliasTable(RenderContext* pRenderContext, const std::vector<ref<Light>>& lights, std::mt19937& rng);

    /** Create a 1D texture with random offsets within a unit circle around (0,0).
        The texture is RG8Snorm for compactness and has no mip maps.
        \param[in] sampleCount Number of samples in the offset texture.
    */
    ref<Texture> createNeighborOffsetTexture(uint32_t sampleCount);

    ref<Scene> mpScene; ///< Scene.

    std::mt19937 mRng; ///< Random generator.

    ref<Device> mpDevice; ///< GPU device.

    ref<PixelDebug> mpPixelDebug; ///< Pixel debug component.

    uint2 mFrameDim = uint2(0); ///< Current frame dimensions.
    uint32_t mFrameIndex = 0;   ///< Current frame index.

    uint32_t mReSTIRInstanceIndex = 0; ///< The index of the ReSTIR instance, used as initial mFrameIndex
    uint32_t mNumReSTIRInstances = 1;  ///< Number of ReSTIR instances that are executed together

    ref<ComputePass> mpReflectTypes; ///< Pass for reflecting types.

    // ReSTIR DI passes.
    ref<ComputePass> mpUpdateEmissiveTriangles; ///< Pass for updating the local emissive triangle data.
    ref<ComputePass> mpGenerateLightTiles;      ///< Pass for generating the light tiles.
    ref<ComputePass> mpInitialResampling;       ///< Pass for initial resampling.
    ref<ComputePass> mpTemporalResampling;      ///< Pass for temporal resampling.
    ref<ComputePass> mpSpatialResampling;       ///< Pass for spatial resampling.
    ref<ComputePass> mpEvaluateFinalSamples;    ///< Pass for evaluating the final samples.

    // ReSTIR GI passes.
    ref<ComputePass> mpGIClearReservoirs; ///< Pass for clearing reservoirs.
    ref<ComputePass> mpGIResampling;      ///< Pass for spatio-temporal resampling.

    // ReSTIR DI resources.
    ref<Buffer> mpEnvLightLuminance; ///< Buffer with luminance values of the env map.
    float mEnvLightLuminanceFactor;  ///< Scalar luminance factor based on env map intensity and tint.
    ref<Buffer> mpEmissiveTriangles; ///< Buffer with emissive triangle data.

    ref<AliasTable> mpEnvLightAliasTable;      ///< Alias table for sampling the env map.
    ref<AliasTable> mpEmissiveLightAliasTable; ///< Alias table for sampling emissive lights.
    ref<AliasTable> mpAnalyticLightAliasTable; ///< Alias table for sampling analytic lights.

    ref<Buffer> mpSurfaceData;     ///< Buffer with the current frame surface data (GBuffer).
    ref<Buffer> mpPrevSurfaceData; ///< Buffer with the previous frame surface data (GBuffer).
    ref<Buffer> mpFinalSamples;    ///< Buffer with the final samples.

    ref<Texture> mpNormalDepthTexture;     ///< Compact normal/depth texture used for fast neighbor pixel validation.
    ref<Texture> mpPrevNormalDepthTexture; ///< Compact normal/depth texture used for fast neighbor pixel validation.
    ref<Texture> mpDebugOutputTexture;     ///< Debug output texture.

    ref<Buffer> mpLightTileData; ///< Buffer with the light tiles (light samples).

    ref<Buffer> mpReservoirs;     ///< Buffer containing the current reservoirs.
    ref<Buffer> mpPrevReservoirs; ///< Buffer containing the previous reservoirs.

    ref<Texture> mpNeighborOffsets; ///< 1D texture containing neighbor offsets within a unit circle.

    // ReSTIR GI resources.
    ref<Buffer> mpGIInitialSamples; ///< 2D Buffer containing initial path tracing samples.
    ref<Buffer> mpGIReservoirs[2];  ///< 2D Buffers containing GI reservoirs.
    float3 mPrevCameraOrigin;       ///< Last frame's camera origin
    float4x4 mPrevViewProj;         ///< Last frame's view projection matrix

    bool mRecompile = true; ///< Recompile programs on next frame if set to true.
    bool mRequestReallocate = false;
    bool mResetTemporalReservoirs = true; ///< Reset temporal reservoir buffer on next frame if set to true.

    int mCurRISPass = 0;
    int mTotalRISPasses = 0;

    struct
    {
        float envLight = 0.f;
        float emissiveLights = 0.f;
        float analyticLights = 0.f;

        /** Compute a discrete set of sample counts given the current selection probabilities.
         */
        std::tuple<uint32_t, uint32_t, uint32_t> getSampleCount(uint32_t totalCount)
        {
            uint32_t envCount = (uint32_t)std::floor(envLight * totalCount);
            uint32_t emissiveCount = (uint32_t)std::floor(emissiveLights * totalCount);
            uint32_t analyticCount = (uint32_t)std::floor(analyticLights * totalCount);
            if (envCount > 0)
                envCount = totalCount - emissiveCount - analyticCount;
            else if (emissiveCount > 0)
                emissiveCount = totalCount - envCount - analyticCount;
            else if (analyticCount > 0)
                analyticCount = totalCount - envCount - emissiveCount;
            return {envCount, emissiveCount, analyticCount};
        }
    } mLightSelectionProbabilities;
};
} // namespace Falcor
