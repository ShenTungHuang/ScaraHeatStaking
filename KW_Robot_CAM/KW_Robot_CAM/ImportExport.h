#pragma once

#include <Storage_Error.hxx>
#include <MgtBRep_TriangleMode.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Quantity_HArray1OfColor.hxx>
#include <TColStd_HArray1OfReal.hxx>

#include <Standard_Macro.hxx>
#include <vector>
#include <AIS_Shape.hxx>



class CImportExport
{
public:
	CImportExport(void);
	~CImportExport(void);

//BRep
//======================================================================
static void ReadBREP(const Handle(AIS_InteractiveContext)& anInteractiveContext);
static Handle(TopTools_HSequenceOfShape) ReadBREP();
static Standard_Boolean ReadBREP(const Standard_CString& aFileName, TopoDS_Shape& aShape);

//Step
//======================================================================
static bool ReadSTEP(const Handle(AIS_InteractiveContext)& anInteractiveContext);
static Handle(TopTools_HSequenceOfShape) ReadSTEP();
static IFSelect_ReturnStatus ReadSTEP(const Standard_CString& aFileName,
									        Handle(TopTools_HSequenceOfShape)& aHSequenceOfShape);
static IFSelect_ReturnStatus ReadSTEP(const Standard_CString& aFileName,
									  std::vector<Handle(AIS_Shape)>& aHSequenceOfAIS_Shape);

public:
	//std::vector<Handle(AIS_Shape)> m_workpiece;

};
