from falcor import *
import os


def render_graph_ScreenSpaceReSTIR():
    g = RenderGraph("ScreenSpaceReSTIR")
    # loadRenderPassLibrary("AccumulatePass.dll")
    # loadRenderPassLibrary("GBuffer.dll")
    # # loadRenderPassLibrary("ReSTIRPTPass.dll")
    # loadRenderPassLibrary("ToneMapper.dll")
    # loadRenderPassLibrary("ScreenSpaceReSTIRPass.dll")
    # loadRenderPassLibrary("ErrorMeasurePass.dll")
    # loadRenderPassLibrary("ImageLoader.dll")

    # ReSTIRGIPlusPass = createPass("ReSTIRPTPass", {'samplesPerPixel': 1})
    # g.addPass(ReSTIRGIPlusPass, "ReSTIRPTPass")
    # VBufferRT = createPass("VBufferRT", {'samplePattern': SamplePattern.Center, 'sampleCount': 1, 'texLOD': TexLODMode.Mip0, 'useAlphaTest': True})
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Center', 'sampleCount': 1, 'texLOD': 'Mip0', 'useAlphaTest': True})
    g.addPass(VBufferRT, "VBufferRT")
    # AccumulatePass = createPass("AccumulatePass", {'enableAccumulation': False, 'precisionMode': AccumulatePrecision.Double})
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    # ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0, 'operator': ToneMapOp.Linear})
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0, 'operator': 'Linear'})
    g.addPass(ToneMapper, "ToneMapper")
    ScreenSpaceReSTIRPass = createPass("ScreenSpaceReSTIRPass")    
    g.addPass(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass")    
    ReSTIRPTPass = createPass("ReSTIRPTPass")    
    g.addPass(ReSTIRPTPass, "ReSTIRPTPass")
    
    # g.addEdge("VBufferRT.vbuffer", "ReSTIRPTPass.vbuffer")   
    # g.addEdge("VBufferRT.mvec", "ReSTIRPTPass.motionVectors")    
    
    g.addEdge("VBufferRT.vbuffer", "ScreenSpaceReSTIRPass.vbuffer")   
    g.addEdge("VBufferRT.mvec", "ScreenSpaceReSTIRPass.motionVectors")    
    # g.addEdge("ScreenSpaceReSTIRPass.color", "ReSTIRPTPass.directLighting")    
    
    # g.addEdge("ReSTIRPTPass.color", "AccumulatePass.input")
    g.addEdge("ScreenSpaceReSTIRPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    
    g.markOutput("ToneMapper.dst")
    g.markOutput("AccumulatePass.output")  
    g.markOutput("ScreenSpaceReSTIRPass.color")  

    return g

graph_ScreenSpaceReSTIR = render_graph_ScreenSpaceReSTIR()

m.addGraph(graph_ScreenSpaceReSTIR)
