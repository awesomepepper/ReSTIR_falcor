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
#pragma once
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "Rendering/ScreenSpaceReSTIR/ScreenSpaceReSTIR.h"
#include "RenderGraph/RenderPassStandardFlags.h"

using namespace Falcor;

class ScreenSpaceReSTIRPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass", "Standalone pass for direct lighting with screen-space ReSTIR.");

    // 原Demo中的create函数用了默认参数，这里没用
    static ref<ScreenSpaceReSTIRPass> create(ref<Device> pDevice, const Properties& props)
    {
        return make_ref<ScreenSpaceReSTIRPass>(pDevice, props);
    }

    // 从构造函数的对比可以看出，Properties取代了Dictionary
    ScreenSpaceReSTIRPass(ref<Device> pDevice, const Properties& props);

    // 基类的getDesc()函数写好了直接从FALCOR_PLUGIN_CLASS里获取描述
    // virtual std::string getDesc() override;
    // 由getProperties函数替代
    // virtual Dictionary getScriptingDictionary() override;

    //virtual void setProperties(const Properties& props) override;
    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override;
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

    void updateProperties(const Properties& props);
    //virtual void updateDict(const Dictionary& dict) override;

private:
    //ScreenSpaceReSTIRPass(const Dictionary& dict);

    //void parseDictionary(const Dictionary& dict);
    void parseProperties(const Properties& props);

    void prepareSurfaceData(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, int instanceID);
    //void validateOptions();
    void finalShading(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, const RenderData& renderData, int instanceID);

    // Internal state
    ref<Scene> mpScene;
    std::vector<ScreenSpaceReSTIR::SharedPtr> mpScreenSpaceReSTIR;
    //std::vector<ScreenSpaceReSTIR::SharedPtr> mpScreenSpaceReSTIR;
    ScreenSpaceReSTIR::Options::SharedPtr mOptions;
    bool mOptionsChanged = false;
    uint2 mFrameDim = uint2(0);
    bool mGBufferAdjustShadingNormals = false;

    ref<ComputePass> mpPrepareSurfaceData;
    //ComputePass::SharedPtr mpPrepareSurfaceData;
    ref<ComputePass> mpFinalShading;

    int mNumReSTIRInstances = 1;
    bool mNeedRecreateReSTIRInstances = false;
};
