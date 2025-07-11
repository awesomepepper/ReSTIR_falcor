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
#include "ScreenSpaceReSTIRPass.h"
#include "RenderGraph/RenderPassHelpers.h"
// git test
namespace
{
const char kDesc[] = "Standalone pass for direct lighting with screen-space ReSTIR.";

const std::string kPrepareSurfaceDataFile = "RenderPasses/ScreenSpaceReSTIRPass/PrepareSurfaceData.cs.slang";
const std::string kFinalShadingFile = "RenderPasses/ScreenSpaceReSTIRPass/FinalShading.cs.slang";

const std::string kInputVBuffer = "vbuffer";
const std::string kInputMotionVectors = "motionVectors";
const std::string kFrameDim = "frameDim";

const std::string kScreenSpaceReSTIRVar = "screenSpaceReSTIR";
const std::string kNumReSTIRInstancesVar = "numReSTIRInstances";
const std::string kReSTIRInstanceIDVar = "ReSTIRInstanceID";
const std::string kDebugChannel = "debug";
const std::string kEnableScreenSpaceReSTIR = "enableScreenSpaceReSTIR";

const std::string kSceneVar = "gScene";
const std::string kPrepareSurfaceDataCB = "gPrepareSurfaceData";
const std::string kFinalShadingCB = "gFinalShading";

const std::string kOutputColor = "color";
const std::string kOutputGColor = "gColor";
const std::string kOutputEmission = "emission";
const std::string kOutputGEmission = "gEmission";
const std::string kOutputDiffuseIllumination = "diffuseIllumination";
const std::string kOutputGDiffuseIllumination = "gDiffuseIllumination";
const std::string kOutputDiffuseReflectance = "diffuseReflectance";
const std::string kOutputGDiffuseReflectance = "gDiffuseReflectance";
const std::string kOutputSpecularIllumination = "specularIllumination";
const std::string kOutputGSpecularIllumination = "gSpecularIllumination";
const std::string kOutputSpecularReflectance = "specularReflectance";
const std::string kOutputGSpecularReflectance = "gSpecularReflectance";
const std::string kOutputDebug = "debug";
const std::string kOutputGDebug = "gDebug";

const Falcor::ChannelList kInputChannels = {
    {kInputVBuffer, "gVBuffer", "Visibility buffer in packed format", false, ResourceFormat::Unknown},
    {kInputMotionVectors, "gMotionVectors", "Motion vector buffer (float format)", true /* optional */, ResourceFormat::RG32Float},
};

const Falcor::ChannelList kOutputChannels = {
    {kOutputColor, kOutputGColor, "Final color", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputEmission, kOutputGEmission, "Emissive color", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputDiffuseIllumination, kOutputGDiffuseIllumination, "Diffuse illumination", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputDiffuseReflectance, kOutputGDiffuseReflectance, "Diffuse reflectance", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputSpecularIllumination, kOutputGSpecularIllumination, "Specular illumination", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputSpecularReflectance, kOutputGSpecularReflectance, "Specular reflectance", true /* optional */, ResourceFormat::RGBA32Float},
    {kOutputDebug, kOutputGDebug, "Debug output", true /* optional */, ResourceFormat::RGBA32Float},
    //{"color", "gColor", "Final color", true /* optional */, ResourceFormat::RGBA32Float},
    //{"emission", "gEmission", "Emissive color", true /* optional */, ResourceFormat::RGBA32Float},
    //{"diffuseIllumination", "gDiffuseIllumination", "Diffuse illumination", true /* optional */, ResourceFormat::RGBA32Float},
    //{"diffuseReflectance", "gDiffuseReflectance", "Diffuse reflectance", true /* optional */, ResourceFormat::RGBA32Float},
    //{"specularIllumination", "gSpecularIllumination", "Specular illumination", true /* optional */, ResourceFormat::RGBA32Float},
    //{"specularReflectance", "gSpecularReflectance", "Specular reflectance", true /* optional */, ResourceFormat::RGBA32Float},
    //{"debug", "gDebug", "Debug output", true /* optional */, ResourceFormat::RGBA32Float},
};

// Scripting options.
const char* kOptions = "options";
const char* kNumReSTIRInstances = "NumReSTIRInstances";
} // namespace

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, ScreenSpaceReSTIRPass>();
}

ScreenSpaceReSTIRPass::ScreenSpaceReSTIRPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);
}

// void ScreenSpaceReSTIRPass::setProperties(const Properties& props)
//{
//     parseProperties(props);
//     validateOptions();
//     if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
//         lightBVHSampler->setOptions(mLightBVHOptions);
//     if (mpRTXDI)
//         mpRTXDI->setOptions(mRTXDIOptions);
//     mRecompile = true;
//     mOptionsChanged = true;
// }

void ScreenSpaceReSTIRPass::parseProperties(const Properties& props)
{
    // ScreenSpaceReSTIR::Options options;
    for (const auto& [key, value] : props)
    {
        if (key == kOptions)
        {
            mOptions = ScreenSpaceReSTIR::Options::create(value);
            // options = value;
        }
        else if (key == kNumReSTIRInstances)
            mNumReSTIRInstances = value;
        else
            logWarning("Unknown field '" + key + "' in ScreenSpaceReSTIRPass dictionary");
    }
    mOptions = ScreenSpaceReSTIR::Options::create();
    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
        mpScreenSpaceReSTIR[0]->mOptions = mOptions;
}

Properties ScreenSpaceReSTIRPass::getProperties() const
{
    Properties props;
    props[kOptions] = mOptions->toProperties();
    props[kNumReSTIRInstances] = mNumReSTIRInstances;
    return props;
}

RenderPassReflection ScreenSpaceReSTIRPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;

    // reflector.addOutput("dst");
    // reflector.addInput("src");
    addRenderPassOutputs(reflector, kOutputChannels);
    addRenderPassInputs(reflector, kInputChannels);

    return reflector;
}

void ScreenSpaceReSTIRPass::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    mFrameDim = compileData.defaultTexDims;
}

void ScreenSpaceReSTIRPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    mpPrepareSurfaceData = nullptr;
    mpFinalShading = nullptr;

    if (!mpScreenSpaceReSTIR.empty())
    {
        // mOptions = mpScreenSpaceReSTIR->getOptions();
        mpScreenSpaceReSTIR.clear();
    }

    if (mpScene)
    {
        // if (is_set(pScene->getPrimitiveTypes(), PrimitiveTypeFlags::Procedural))
        //{
        //     logError("This render pass does not support procedural primitives such as curves.");
        // }

        mpScreenSpaceReSTIR.resize(mNumReSTIRInstances);
        for (int i = 0; i < mNumReSTIRInstances; i++)
        {
            // 这里mpDevice这个参数是后加的，原demo中ScreenSpaceReSTIR的create函数没这个参数
            mpScreenSpaceReSTIR[i] = ScreenSpaceReSTIR::create(mpDevice, mpScene, mOptions, mNumReSTIRInstances, i);
        }
    }
}

bool ScreenSpaceReSTIRPass::onMouseEvent(const MouseEvent& mouseEvent)
{
    return !mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0] ? mpScreenSpaceReSTIR[0]->getPixelDebug()->onMouseEvent(mouseEvent)
                                                                  : false;
}

void ScreenSpaceReSTIRPass::updateProperties(const Properties& props)
{
    parseProperties(props);
    mOptionsChanged = true;
    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
        mpScreenSpaceReSTIR[0]->resetReservoirCount();
}

void ScreenSpaceReSTIRPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData.getTexture("src");

    if (mNeedRecreateReSTIRInstances)
        setScene(pRenderContext, mpScene);

    const auto& pVBuffer = renderData[kInputVBuffer]->asTexture();
    const auto& pMotionVectors = renderData[kInputMotionVectors]->asTexture();

    // Clear outputs if ReSTIR module is not initialized.
    if (mpScreenSpaceReSTIR.empty())
    {
        auto clear = [&](const ChannelDesc& channel)
        {
            if (renderData[channel.name] != nullptr)
            {
                auto pTex = renderData[channel.name]->asTexture();
                if (pTex)
                    pRenderContext->clearUAV(pTex->getUAV().get(), float4(0.f));
            }
            // auto pTex = renderData[channel.name]->asTexture();
            // if (pTex)
            //     pRenderContext->clearUAV(pTex->getUAV().get(), float4(0.f));
        };
        for (const auto& channel : kOutputChannels)
            clear(channel);
        return;
    }

    auto& dict = renderData.getDictionary();

    if (dict.keyExists(kEnableScreenSpaceReSTIR))
    {
        for (int i = 0; i < mpScreenSpaceReSTIR.size(); i++)
            mpScreenSpaceReSTIR[i]->enablePass((bool)dict[kEnableScreenSpaceReSTIR]);
    }

    // Update refresh flag if changes that affect the output have occured.
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
        mOptionsChanged = false;
    }

    // Check if GBuffer has adjusted shading normals enabled.
    mGBufferAdjustShadingNormals = dict.getValue(Falcor::kRenderPassGBufferAdjustShadingNormals, false);

    for (int i = 0; i < mpScreenSpaceReSTIR.size(); i++)
    {
        mpScreenSpaceReSTIR[i]->beginFrame(pRenderContext, mFrameDim);

        prepareSurfaceData(pRenderContext, pVBuffer, i);

        mpScreenSpaceReSTIR[i]->updateReSTIRDI(pRenderContext, pMotionVectors);

        finalShading(pRenderContext, pVBuffer, renderData, i);

        mpScreenSpaceReSTIR[i]->endFrame(pRenderContext);
    }

    auto copyTexture = [pRenderContext](Texture* pDst, const Texture* pSrc)
    {
        if (pDst && pSrc)
        {
            assert(pDst && pSrc);
            assert(pDst->getFormat() == pSrc->getFormat());
            assert(pDst->getWidth() == pSrc->getWidth() && pDst->getHeight() == pSrc->getHeight());
            pRenderContext->copyResource(pDst, pSrc);
        }
        else if (pDst)
        {
            pRenderContext->clearUAV(pDst->getUAV().get(), uint4(0, 0, 0, 0));
        }
    };
    // Copy debug output if available. (only support first ReSTIR instance for now)
    if (const auto& pDebug = renderData["debug"]->asTexture())
    {
        copyTexture(pDebug.get(), mpScreenSpaceReSTIR[0]->getDebugOutputTexture().get());
    }
}

void ScreenSpaceReSTIRPass::renderUI(Gui::Widgets& widget)
{
    mNeedRecreateReSTIRInstances = widget.var("Num ReSTIR Instances", mNumReSTIRInstances, 1, 8);

    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
    {
        mOptionsChanged = mpScreenSpaceReSTIR[0]->renderUI(widget);
        for (int i = 1; i < mpScreenSpaceReSTIR.size(); i++)
            mpScreenSpaceReSTIR[i]->copyRecompileStateFromOtherInstance(mpScreenSpaceReSTIR[0]);
    }
}

void ScreenSpaceReSTIRPass::prepareSurfaceData(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, int instanceID)
{
    assert(!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[instanceID]);

    // PROFILE("prepareSurfaceData");
    FALCOR_PROFILER(pRenderContext, "prepareSurfaceData");

    if (!mpPrepareSurfaceData)
    {
        auto defines = mpScene->getSceneDefines();
        defines.add("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
        mpPrepareSurfaceData = ComputePass::create(mpDevice, kPrepareSurfaceDataFile, "main", defines, false);
        mpPrepareSurfaceData->setVars(nullptr);
    }

    mpPrepareSurfaceData->addDefine("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");

    // mpPrepareSurfaceData["gScene"] = mpScene->getParameterBlock();
    mpScene->bindShaderData(mpPrepareSurfaceData->getRootVar()[kSceneVar]);

    // auto var = mpPrepareSurfaceData["CB"]["gPrepareSurfaceData"];
    auto var = mpPrepareSurfaceData->getRootVar()[kPrepareSurfaceDataCB];

    var["vbuffer"] = pVBuffer;
    var["frameDim"] = mFrameDim;
    mpScreenSpaceReSTIR[instanceID]->setShaderData(var[kScreenSpaceReSTIRVar]);

    if (instanceID == 0 && mpFinalShading && mpScreenSpaceReSTIR[0]->mRequestParentRecompile)
    {
        mpFinalShading->setVars(nullptr);
        mpScreenSpaceReSTIR[0]->mRequestParentRecompile = false;
    }

    mpPrepareSurfaceData->execute(pRenderContext, mFrameDim.x, mFrameDim.y);
}

void ScreenSpaceReSTIRPass::finalShading(
    RenderContext* pRenderContext,
    const ref<Texture>& pVBuffer,
    const RenderData& renderData,
    int instanceID
)
{
    assert(!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[instanceID]);

    // PROFILE("finalShading");
    FALCOR_PROFILER(pRenderContext, "finalShading");

    if (!mpFinalShading)
    {
        auto defines = mpScene->getSceneDefines();
        defines.add("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
        // defines.add("USE_ENV_BACKGROUND", mpScene->useEnvBackground() ? "1" : "0");
        defines.add(getValidResourceDefines(kOutputChannels, renderData));
        mpFinalShading = ComputePass::create(mpDevice, kFinalShadingFile, "main", defines, false);
        mpFinalShading->setVars(nullptr);
    }

    mpFinalShading->addDefine("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
    // mpFinalShading->addDefine("USE_ENV_BACKGROUND", mpScene->useEnvBackground() ? "1" : "0");
    mpFinalShading->addDefine("_USE_LEGACY_SHADING_CODE", "0");

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    mpFinalShading->getProgram()->addDefines(getValidResourceDefines(kOutputChannels, renderData));

    // mpFinalShading["gScene"] = mpScene->getParameterBlock();
    mpScene->bindShaderData(mpFinalShading->getRootVar()[kSceneVar]);

    // auto var = mpFinalShading["CB"]["gFinalShading"];
    auto var = mpFinalShading->getRootVar()["gFinalShading"];

    var["vbuffer"] = pVBuffer;
    var["frameDim"] = mFrameDim;
    var["numReSTIRInstances"] = mNumReSTIRInstances;
    var["ReSTIRInstanceID"] = instanceID;

    mpScreenSpaceReSTIR[instanceID]->setShaderData(var[kScreenSpaceReSTIRVar]);

    // Bind output channels as UAV buffers.
    var = mpFinalShading->getRootVar();
    auto bind = [&](const ChannelDesc& channel)
    {
        ref<Texture> pTex = renderData[channel.name]->asTexture();
        var[channel.texname] = pTex;
    };
    for (const auto& channel : kOutputChannels)
        bind(channel);

    mpFinalShading->execute(pRenderContext, mFrameDim.x, mFrameDim.y);
}
