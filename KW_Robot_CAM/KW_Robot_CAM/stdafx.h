// stdafx.h : 可在此標頭檔中包含標準的系統 Include 檔，
// 或是經常使用卻很少變更的
// 專案專用 Include 檔案

#if !defined(AFX_STDAFX_H__4A96A446_055A_11D4_A60A_0800369C8A03__INCLUDED_)
#define AFX_STDAFX_H__4A96A446_055A_11D4_A60A_0800369C8A03__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#define _CRT_SECURE_NO_DEPRECATE

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#include <afxdisp.h>        // MFC OLE automation classes
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT

// Open CASCADE Technology :

#if !defined(WNT)
   #error WNT precompiler directive is mandatory for Open CASCADE Technology
#endif
#if !defined(CSFDB)
   #error CSFDB precompiler directive is mandatory for Open CASCADE Technology
#endif

#ifdef MAINWIN
# define false FALSE
# define true TRUE
#endif

#pragma warning(  disable : 4244 )        // Issue warning 4244
#include <Standard_ShortReal.hxx>
#pragma warning(  default : 4244 )        // Issue warning 4244

#include <AIS_Drawer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_SequenceOfInteractive.hxx>
#include <AIS_Trihedron.hxx>

#include <Aspect_Background.hxx>
#include <Aspect_ColorMapEntry.hxx>
#include <Aspect_GenericColorMap.hxx>
#include <Aspect_LineStyle.hxx>
#include <Aspect_MarkMap.hxx>
#include <Aspect_TypeMap.hxx>
#include <Aspect_TypeMapEntry.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Aspect_TypeOfText.hxx>
#include <Aspect_WidthMapEntry.hxx>
#include <Aspect_WidthOfLine.hxx>
#include <Aspect_Window.hxx>
#include <Bnd_Box2d.hxx>
#include <BndLib_Add2dCurve.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>

#include <BRepBuilderAPI.hxx>
#include <BRepAlgo.hxx>
#include <BRepAdaptor_Curve.hxx>

#include <BRepTools.hxx>
#include <Standard_DefineHandle.hxx> 
#include <DsgPrs_LengthPresentation.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Geom_Line.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomTools_Curve2dSet.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_WNTGraphicDevice.hxx>
#include <MMgt_TShared.hxx>
#include <OSD_Environment.hxx>
#include <Precision.hxx>
#include <Prs3d_IsoAspect.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_Projector.hxx>
#include <Prs3d_Text.hxx>
#include <PrsMgr_PresentationManager2d.hxx>
#include <Quantity_Factor.hxx>
#include <Quantity_Length.hxx>
#include <Quantity_NameOfColor.hxx>
#include <Quantity_PhysicalQuantity.hxx>
#include <Quantity_PlaneAngle.hxx>
#include <Quantity_TypeOfColor.hxx>
#include <SelectBasics_BasicTool.hxx>
#include <SelectBasics_ListOfBox2d.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <SelectMgr_Selection.hxx>
#include <SelectMgr_SelectionManager.hxx>
#include <ShapeSchema.hxx> 
#include <Standard_Boolean.hxx>
#include <Standard_CString.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Integer.hxx>
#include <Standard_IStream.hxx>
#include <Standard_Macro.hxx>
#include <Standard_NotImplemented.hxx>
#include <Standard_OStream.hxx>
#include <Standard_Real.hxx>
#include <StdPrs_Curve.hxx>
#include <StdPrs_Point.hxx>
#include <StdPrs_PoleCurve.hxx>
#include <StdSelect_SensitiveText2d.hxx>
#include <StdSelect_TextProjector2d.hxx>
#include <StdSelect_ViewerSelector2d.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColgp_HArray1OfPnt2d.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
#include <TColStd_MapIteratorOfMapOfTransient.hxx>
#include <TColStd_MapOfTransient.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_ListIteratorOfListOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <UnitsAPI.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_WDriver.hxx>
#include <WNT_Window.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS_Edge.hxx>

//import export
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <TopoDS_Builder.hxx>


//Light
#include <V3d_AmbientLight.hxx>

// cool tab
#include ".\CSCB\sizecbar.h"、
#include ".\CSCB\scbarg.h"、
#include ".\CSCB\scbarcf.h"、
#include ".\CSCB\CoolTabCtrl.h"


#include <BRepBuilderAPI_Transform.hxx>



#endif // !defined(AFX_STDAFX_H__4A96A446_055A_11D4_A60A_0800369C8A03__INCLUDED_)
