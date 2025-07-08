add_plugin(ReSTIRPTPass)

target_sources(ReSTIRPTPass PRIVATE
    ReSTIRPTPass.cpp
    ReSTIRPTPass.h
    TemporalPathRetrace.cs.slang
    TemporalReuse.cs.slang
    TracePass.cs.slang
    SpatialPathRetrace.cs.slang
    SpatialReuse.cs.slang
    StaticParams.slang
    ReflectTypes.cs.slang
    Shift.slang
    PathTracer.slang
    PathBuilder.slang
    PathReservoir.slang
    PathState.slang
    GeneratePaths.cs.slang
    LoadShadingData.slang
    NRDHelpers.slang
    Params.slang
    ComputePathReuseMISWeights.cs.slang
)

target_copy_shaders(PathTracer RenderPasses/ReSTIRPTPass)

target_source_group(ReSTIRPTPass "RenderPasses")
