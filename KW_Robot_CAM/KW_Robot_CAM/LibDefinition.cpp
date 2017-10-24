#include "stdafx.h"

#include <Standard_Version.hxx>

#pragma message ("============== Set libraries for Open CASCADE Technology " OCC_VERSION_STRING)

#pragma comment (lib , "TKBool.lib")
#pragma comment (lib , "TKBRep.lib")
#pragma comment (lib , "TKernel.lib")
#pragma comment (lib , "TKG2d.lib")
#pragma comment (lib , "TKG3d.lib")
#pragma comment (lib , "TKMath.lib")
#pragma comment (lib , "TKService.lib")
#pragma comment (lib , "TKV3d.lib")
#pragma comment (lib , "TKTopAlgo.lib")
#pragma comment (lib , "TKPrim.lib")
#pragma comment (lib , "TKBO.lib")


//ImportExport
#pragma comment (lib , "PTKernel.lib")
#pragma comment (lib , "TKXSBase.lib")
#pragma comment (lib , "TKStep.lib")
#pragma comment (lib , "TKSTEPAttr.lib")  // TopoDS_Shape S = STEPConstruct::FindShape(aStyles.TransientProcess(), style->Item());
#pragma comment (lib , "TKSTEPBase.lib")  // Handle(StepShape_ShapeRepresentation) aCurrentSR = Handle(StepShape_ShapeRepresentation)::DownCast(aStyleCntxSlct.Representation());
#pragma comment (lib , "TKXDESTEP.lib")   //STEPCAFControl_Reader
#pragma comment (lib , "TKXCAF.lib")
#pragma comment (lib , "TKXCAFSchema.lib")
#pragma comment (lib , "TKLCAF.lib")