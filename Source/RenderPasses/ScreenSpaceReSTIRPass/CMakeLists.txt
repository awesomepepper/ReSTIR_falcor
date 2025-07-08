add_plugin(ScreenSpaceReSTIRPass)

target_sources(ScreenSpaceReSTIRPass PRIVATE
    ScreenSpaceReSTIRPass.cpp
    ScreenSpaceReSTIRPass.h
    FinalShading.cs.slang
    LoadShadingData.slang
    PrepareSurfaceData.cs.slang
)

target_source_group(ScreenSpaceReSTIRPass "RenderPasses")
