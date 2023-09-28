'# MWS Version: Version 2020.7 - Jul 10 2020 - ACIS 29.0.1 -

'# length = mm
'# frequency = GHz
'# time = ns
'# frequency range: fmin = 4 fmax = 6
'# created = '[VERSION]2020.7|29.0.1|20200710[/VERSION]


'@ use template: Antenna - Planar.cfg

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
'set the units
With Units
    .Geometry "mm"
    .Frequency "GHz"
    .Voltage "V"
    .Resistance "Ohm"
    .Inductance "NanoH"
    .TemperatureUnit  "Kelvin"
    .Time "ns"
    .Current "A"
    .Conductance "Siemens"
    .Capacitance "PikoF"
End With
'----------------------------------------------------------------------------
'set the frequency range
Solver.FrequencyRange "4", "6"
'----------------------------------------------------------------------------
Plot.DrawBox True
With Background
     .Type "Normal"
     .Epsilon "1.0"
     .Mu "1.0"
     .XminSpace "0.0"
     .XmaxSpace "0.0"
     .YminSpace "0.0"
     .YmaxSpace "0.0"
     .ZminSpace "0.0"
     .ZmaxSpace "0.0"
End With
With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "expanded open"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
End With
' optimize mesh settings for planar structures
With Mesh
     .MergeThinPECLayerFixpoints "True"
     .RatioLimit "20"
     .AutomeshRefineAtPecLines "True", "6"
     .FPBAAvoidNonRegUnite "True"
     .ConsiderSpaceForLowerMeshLimit "False"
     .MinimumStepNumber "5"
     .AnisotropicCurvatureRefinement "True"
     .AnisotropicCurvatureRefinementFSM "True"
End With
With MeshSettings
     .SetMeshType "Hex"
     .Set "RatioLimitGeometry", "20"
     .Set "EdgeRefinementOn", "1"
     .Set "EdgeRefinementRatio", "6"
End With
With MeshSettings
     .SetMeshType "HexTLM"
     .Set "RatioLimitGeometry", "20"
End With
With MeshSettings
     .SetMeshType "Tet"
     .Set "VolMeshGradation", "1.5"
     .Set "SrfMeshGradation", "1.5"
End With
' change mesh adaption scheme to energy
' 		(planar structures tend to store high energy
'     	 locally at edges rather than globally in volume)
MeshAdaption3D.SetAdaptionStrategy "Energy"
' switch on FD-TET setting for accurate farfields
FDSolver.ExtrudeOpenBC "True"
PostProcess1D.ActivateOperation "vswr", "true"
PostProcess1D.ActivateOperation "yz-matrices", "true"
With FarfieldPlot
	.ClearCuts ' lateral=phi, polar=theta
	.AddCut "lateral", "0", "1"
	.AddCut "lateral", "90", "1"
	.AddCut "polar", "90", "1"
End With
'----------------------------------------------------------------------------
With MeshSettings
     .SetMeshType "Hex"
     .Set "Version", 1%
End With
With Mesh
     .MeshType "PBA"
End With
'set the solver type
ChangeSolverType("HF Time Domain")
'----------------------------------------------------------------------------

'@ define material: Folder1/FR4

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Material 
     .Reset 
     .Name "FR4"
     .Folder "Folder1"
     .Rho "0.0"
     .ThermalType "Normal"
     .ThermalConductivity "0"
     .SpecificHeat "0", "J/K/kg"
     .DynamicViscosity "0"
     .Emissivity "0"
     .MetabolicRate "0.0"
     .VoxelConvection "0.0"
     .BloodFlow "0"
     .MechanicsType "Unused"
     .FrqType "all"
     .Type "Normal"
     .MaterialUnit "Frequency", "GHz"
     .MaterialUnit "Geometry", "mm"
     .MaterialUnit "Time", "ns"
     .MaterialUnit "Temperature", "Kelvin"
     .Epsilon "3.9"
     .Mu "1"
     .Sigma "0"
     .TanD "0.02"
     .TanDFreq "5"
     .TanDGiven "True"
     .TanDModel "ConstTanD"
     .EnableUserConstTanDModelOrderEps "False"
     .ConstTanDModelOrderEps "1"
     .SetElParametricConductivity "False"
     .ReferenceCoordSystem "Global"
     .CoordSystemType "Cartesian"
     .SigmaM "0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstTanD"
     .EnableUserConstTanDModelOrderMu "False"
     .ConstTanDModelOrderMu "1"
     .SetMagParametricConductivity "False"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "Nth Order"
     .MaximalOrderNthModelFitEps "10"
     .ErrorLimitNthModelFitEps "0.1"
     .UseOnlyDataInSimFreqRangeNthModelEps "False"
     .DispersiveFittingSchemeMu "Nth Order"
     .MaximalOrderNthModelFitMu "10"
     .ErrorLimitNthModelFitMu "0.1"
     .UseOnlyDataInSimFreqRangeNthModelMu "False"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .NLAnisotropy "False"
     .NLAStackingFactor "1"
     .NLADirectionX "1"
     .NLADirectionY "0"
     .NLADirectionZ "0"
     .Colour "1", "0.501961", "1" 
     .Wireframe "False" 
     .Reflection "False" 
     .Allowoutline "True" 
     .Transparentoutline "False" 
     .Transparency "0" 
     .Create
End With

'@ new component: component1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Component.New "component1"

'@ define brick: component1:sustrato

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "sustrato" 
     .Component "component1" 
     .Material "Folder1/FR4" 
     .Xrange "-L_sust/2", "L_sust/2" 
     .Yrange "-W_sust/2", "W_sust/2" 
     .Zrange "0", "h_sust" 
     .Create
End With

'@ define brick: component1:parche

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "parche" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-L/2", "L/2" 
     .Yrange "-W/2", "W/2" 
     .Zrange "h_sust", "h_cobre+h_sust" 
     .Create
End With

'@ define brick: component1:linea

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "linea" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "(-L/2)-L_strip+X0", "-L/2+X0" 
     .Yrange "-W_strip/2", "W_strip/2" 
     .Zrange "h_sust", "h_sust+h_cobre" 
     .Create
End With

'@ define brick: component1:plano_masa

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "plano_masa" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-L_sust/2", "L_sust/2" 
     .Yrange "-W_sust/2", "W_sust/2" 
     .Zrange "0", "0" 
     .Create
End With

'@ define boundaries

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "electric"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
     .ApplyInAllDirections "False"
     .OpenAddSpaceFactor "0.5"
End With

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "4"

'@ define port: 1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "1" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20", "-20" 
     .Yrange "-1", "1" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ define time domain solver parameters

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Mesh.SetCreator "High Frequency" 
With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-S"
     .StimulationPort "All"
     .StimulationMode "All"
     .SteadyStateLimit "-40"
     .MeshAdaption "False"
     .AutoNormImpedance "False"
     .NormingImpedance "50"
     .CalculateModesOnly "True"
     .SParaSymmetry "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ set PBA version

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Discretizer.PBAVersion "2020071020"

'@ define time domain solver parameters

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Mesh.SetCreator "High Frequency" 
With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-S"
     .StimulationPort "All"
     .StimulationMode "All"
     .SteadyStateLimit "-40"
     .MeshAdaption "False"
     .AutoNormImpedance "False"
     .NormingImpedance "50"
     .CalculateModesOnly "False"
     .SParaSymmetry "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ change solver type

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
ChangeSolverType "HF Time Domain"

'@ pick edge

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEdgeFromId "component1:linea", "2", "2"

'@ define distance dimension

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Dimension
    .Reset
    .CreationType "picks"
    .SetType "Distance"
    .SetID "0"
    .SetOrientation "Smart Mode"
    .SetDistance "2.379615"
    .SetViewVector "-0.469844", "-0.342034", "-0.813793"
    .SetConnectedElement1 "component1:linea"
    .SetConnectedElement2 "component1:linea"
    .Create
End With
Pick.ClearAllPicks

'@ define brick: component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "mordisco" 
     .Component "component1" 
     .Material "Vacuum" 
     .Xrange "-L/2", "-L/2+X0" 
     .Yrange "W_strip/2", "W_strip/2+0.5" 
     .Zrange "h_sust", "h_sust+h_cobre" 
     .Create
End With

'@ boolean insert shapes: component1:parche, component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Solid.Insert "component1:parche", "component1:mordisco"

'@ transform: mirror component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco" 
     .Origin "Free" 
     .Center "0", "0", "0" 
     .PlaneNormal "0", "1", "0" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Mirror" 
End With

'@ boolean insert shapes: component1:parche, component1:mordisco_1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Solid.Insert "component1:parche", "component1:mordisco_1"

'@ clear picks

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.ClearAllPicks

'@ define farfield monitor: farfield (f=5)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=5)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "5" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-20.409", "20.409", "-21.62", "21.62", "0", "11.615" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ define farfield monitor: farfield (f=4.9014)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=4.9014)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "4.9014" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-20.409", "20.409", "-21.62", "21.62", "0", "11.615" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ define farfield monitor: farfield (f=5.1033)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=5.1033)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "5.1033" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-20.409", "20.409", "-21.62", "21.62", "0", "11.615" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ switch working plane

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Plot.DrawWorkplane "false"

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "TRUE" 
     .Arraytype "Rectangular" 
     .XSet "1", "0", "0" 
     .YSet "4", "5", "0" 
     .ZSet "1", "0", "0" 
     .SetList
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "FALSE" 
     .Arraytype "Rectangular" 
     .XSet "1", "0", "0" 
     .YSet "4", "5", "0" 
     .ZSet "1", "0", "0" 
     .SetList
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "5.96297" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ delete monitor: farfield (f=4.9014)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Monitor.Delete "farfield (f=4.9014)"

'@ delete monitor: farfield (f=5.1033)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Monitor.Delete "farfield (f=5.1033)"

'@ define farfield monitor: farfield (f=4.9035)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=4.9035)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "4.9035" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-20.409", "20.409", "-20.85", "20.85", "0", "6.615" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ define farfield monitor: farfield (f=5.1004)

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=5.1004)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "5.1004" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-20.409", "20.409", "-20.85", "20.85", "0", "6.615" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Cartesian" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "TRUE" 
     .Arraytype "Rectangular" 
     .XSet "1", "0", "0" 
     .YSet "4", "5", "0" 
     .ZSet "1", "0", "0" 
     .SetList
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "TRUE" 
     .Arraytype "Rectangular" 
     .XSet "1", "0", "0" 
     .YSet "22", "lambda*0.75", "45" 
     .ZSet "1", "0", "0" 
     .SetList
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "TRUE" 
     .Arraytype "Edit" 
     .XSet "1", "0", "0" 
     .YSet "26", "lambda*0.8", "50" 
     .ZSet "1", "0", "0" 
     .Antenna "0", "-600", "0", "0.1", "-265" 
     .Antenna "0", "-552", "0", "0.2", "-215" 
     .Antenna "0", "-504", "0", "0.3", "-165" 
     .Antenna "0", "-456", "0", "0.4", "-115" 
     .Antenna "0", "-408", "0", "0.5", "-65" 
     .Antenna "0", "-360", "0", "0.6", "-15" 
     .Antenna "0", "-312", "0", "0.7", "-325" 
     .Antenna "0", "-264", "0", "0.8", "-275" 
     .Antenna "0", "-216", "0", "0.9", "-225" 
     .Antenna "0", "-168", "0", "1", "-175" 
     .Antenna "0", "-120", "0", "1", "-125" 
     .Antenna "0", "-72", "0", "1", "-75" 
     .Antenna "0", "-24", "0", "1", "-25" 
     .Antenna "0", "24", "0", "1", "25" 
     .Antenna "0", "72", "0", "1", "75" 
     .Antenna "0", "120", "0", "1", "125" 
     .Antenna "0", "168", "0", "1", "175" 
     .Antenna "0", "216", "0", "0.9", "225" 
     .Antenna "0", "264", "0", "0.8", "275" 
     .Antenna "0", "312", "0", "0.7", "325" 
     .Antenna "0", "360", "0", "0.6", "15" 
     .Antenna "0", "408", "0", "0.5", "65" 
     .Antenna "0", "456", "0", "0.4", "115" 
     .Antenna "0", "504", "0", "0.3", "165" 
     .Antenna "0", "552", "0", "0.2", "215" 
     .Antenna "0", "600", "0", "0.1", "265" 
End With

'@ farfield plot options

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  
     .StoreSettings
End With

'@ farfield array properties

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With FarfieldArray
     .Reset 
     .UseArray "FALSE" 
     .Arraytype "Edit" 
     .XSet "1", "0", "0" 
     .YSet "26", "lambda*0.8", "50" 
     .ZSet "1", "0", "0" 
     .Antenna "0", "-600", "0", "0.11", "-265" 
     .Antenna "0", "-554", "0", "0.21", "-215" 
     .Antenna "0", "-504", "0", "0.31", "-165" 
     .Antenna "0", "-456", "0", "0.4", "-115" 
     .Antenna "0", "-408", "0", "0.5", "-65" 
     .Antenna "0", "-360", "0", "0.6", "-15" 
     .Antenna "0", "-312", "0", "0.7", "-325" 
     .Antenna "0", "-264", "0", "0.8", "-275" 
     .Antenna "0", "-216", "0", "0.9", "-225" 
     .Antenna "0", "-168", "0", "1", "-175" 
     .Antenna "0", "-120", "0", "1", "-125" 
     .Antenna "0", "-72", "0", "1", "-75" 
     .Antenna "0", "-24", "0", "1", "-25" 
     .Antenna "0", "24", "0", "1", "25" 
     .Antenna "0", "72", "0", "1", "75" 
     .Antenna "0", "120", "0", "1", "125" 
     .Antenna "0", "168", "0", "1", "175" 
     .Antenna "0", "216", "0", "0.9", "225" 
     .Antenna "0", "264", "0", "0.8", "275" 
     .Antenna "0", "312", "0", "0.7", "325" 
     .Antenna "0", "360", "0", "0.6", "15" 
     .Antenna "0", "408", "0", "0.5", "65" 
     .Antenna "0", "456", "0", "0.4", "115" 
     .Antenna "0", "504", "0", "0.31", "165" 
     .Antenna "0", "552", "0", "0.21", "215" 
     .Antenna "0", "600", "0", "0.11", "265" 
End With

'@ transform: translate component1:linea

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:linea" 
     .Vector "0", "0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco" 
     .Vector "0", "0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco_1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco_1" 
     .Vector "0", "0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:parche

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:parche" 
     .Vector "0", "0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:linea

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:linea" 
     .Vector "0", "0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco" 
     .Vector "0", "0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco_1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco_1" 
     .Vector "0", "0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:parche

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:parche" 
     .Vector "0", "0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:linea

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:linea" 
     .Vector "0", "-0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco" 
     .Vector "0", "-0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco_1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco_1" 
     .Vector "0", "-0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:parche

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:parche" 
     .Vector "0", "-0.86*lambda", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:linea

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:linea" 
     .Vector "0", "-0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco" 
     .Vector "0", "-0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:mordisco_1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:mordisco_1" 
     .Vector "0", "-0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "True" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ transform: translate component1:parche

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Transform 
     .Reset 
     .Name "component1:parche" 
     .Vector "0", "-0.86*lambda*2", "0" 
     .UsePickedPoints "False" 
     .InvertPickedPoints "False" 
     .MultipleObjects "True" 
     .GroupObjects "False" 
     .Repetitions "1" 
     .MultipleSelection "False" 
     .Destination "" 
     .Material "" 
     .Transform "Shape", "Translate" 
End With

'@ switch working plane

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Plot.DrawWorkplane "true"

'@ define brick: component1:medidas

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Brick
     .Reset 
     .Name "medidas" 
     .Component "component1" 
     .Material "PEC" 
     .Xrange "-5", "5" 
     .Yrange "0", "0.86*lambda" 
     .Zrange "h_sust", "h_sust+h_cobre" 
     .Create
End With

'@ pick edge

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEdgeFromId "component1:medidas", "1", "1"

'@ define distance dimension

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Dimension
    .Reset
    .CreationType "picks"
    .SetType "Distance"
    .SetID "1"
    .SetOrientation "Smart Mode"
    .SetDistance "3.653342"
    .SetViewVector "0.671751", "0.196847", "-0.714144"
    .SetConnectedElement1 "component1:medidas"
    .SetConnectedElement2 "component1:medidas"
    .Create
End With
Pick.ClearAllPicks

'@ delete shape: component1:medidas

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Solid.Delete "component1:medidas"

'@ delete port: port1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Port.Delete "1"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_2", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_2", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_2", "4"

'@ define port: 1

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "1" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20.409", "-20.409" 
     .Yrange "102.5", "103.9" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_1", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_1", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_1", "4"

'@ define port: 2

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "2" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20.409", "-20.409" 
     .Yrange "50.9", "52.3" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea", "4"

'@ define port: 3

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "3" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20.409", "-20.409" 
     .Yrange "-0.7", "0.7" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_3", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_3", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_3", "4"

'@ define port: 4

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "4" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20.409", "-20.409" 
     .Yrange "-52.3", "-50.9" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_4", "3"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_4", "8"

'@ pick end point

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Pick.PickEndpointFromId "component1:linea_4", "4"

'@ define port: 5

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
With Port 
     .Reset 
     .PortNumber "5" 
     .Label "" 
     .Folder "" 
     .NumberOfModes "1" 
     .AdjustPolarization "False" 
     .PolarizationAngle "0.0" 
     .ReferencePlaneDistance "0" 
     .TextSize "50" 
     .TextMaxLimit "0" 
     .Coordinates "Picks" 
     .Orientation "positive" 
     .PortOnBound "False" 
     .ClipPickedPortToBound "False" 
     .Xrange "-20.409", "-20.409" 
     .Yrange "-103.9", "-102.5" 
     .Zrange "1.58", "1.615" 
     .XrangeAdd "0.0", "0.0" 
     .YrangeAdd "Y_puerto", "Y_puerto" 
     .ZrangeAdd "1.58", "Z_puerto" 
     .SingleEnded "False" 
     .WaveguideMonitor "False" 
     .Create 
End With

'@ delete monitors

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Monitor.Delete "farfield (f=4.9035)" 
Monitor.Delete "farfield (f=5)" 
Monitor.Delete "farfield (f=5.1004)"

'@ define frequency range

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Solver.FrequencyRange "4", "6"

'@ switch working plane

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Plot.DrawWorkplane "false"

'@ define frequency range

'[VERSION]2020.7|29.0.1|20200710[/VERSION]
Solver.FrequencyRange "4", "6" 


