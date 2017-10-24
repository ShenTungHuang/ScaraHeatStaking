#include "StdAfx.h"
#include "ImportExport.h"

//#include <vector>

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>

#include "TColStd_SequenceOfAsciiString.hxx"
#include "TColStd_SequenceOfExtendedString.hxx"
#include "OSD_Timer.hxx"

#include "IGESControl_Reader.hxx"
#include "STEPControl_Controller.hxx"

#include <STEPCAFControl_Reader.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ShapeTool.hxx>
//#include <Handle_XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDF_ChildIterator.hxx>
#include <XCAFDoc_Centroid.hxx>
#include <XCAFDoc_Location.hxx>

#include <PTColStd_PersistentTransientMap.hxx>
#include <PTColStd_TransientPersistentMap.hxx>

#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>

#include <XSControl_WorkSession.hxx>
#include <STEPConstruct_Styles.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
#include <STEPConstruct.hxx>
#include <StepVisual_StyledItem.hxx>

#ifdef _DEBUG
//#define new DEBUG_NEW
#endif
Handle(AIS_Shape) 	aisWorkpiece;
CString workpiece_name;
//std::vector<Handle(AIS_Shape)> m_workpiece;

CImportExport::CImportExport(void)
{
}

CImportExport::~CImportExport(void)
{
}

//======================================================================
//=                                                                    =
//=                      BREP                                          =
//=                                                                    =
//======================================================================

void CImportExport::ReadBREP(const Handle(AIS_InteractiveContext)& anInteractiveContext)
{
    Handle(TopTools_HSequenceOfShape) aSequence = CImportExport::ReadBREP();
    for(int i=1;i<= aSequence->Length();i++) {
			anInteractiveContext->Display(new AIS_Shape(aSequence->Value(i)));
	}
}

Handle(TopTools_HSequenceOfShape) CImportExport::ReadBREP()
{
  setlocale(LC_ALL,"chinese-traditional");
  CFileDialog dlg(TRUE,
				  NULL,
				  NULL,
				  OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
				  "BREP Files (*.brep, *.rle)|*.brep; *.rle; |All Files (*.*)|*.*||", 
				  NULL );

  Handle(TopTools_HSequenceOfShape) aSequence= new TopTools_HSequenceOfShape();

  if (dlg.DoModal() == IDOK) 
  {
	SetCursor(AfxGetApp()->LoadStandardCursor(IDC_WAIT));
	CString filename = dlg.GetPathName();
    TopoDS_Shape aShape;
    Standard_CString aFileName = (Standard_CString)(LPCTSTR)filename;
	Standard_Boolean result = ReadBREP(aFileName,aShape);
	if (result)
    {
 	    if (!BRepAlgo::IsValid(aShape))
	        MessageBox(0,"warning : The shape is not valid!!!","CasCade Warning",MB_ICONWARNING);
       aSequence->Append(aShape);
    }
    else 
	   MessageBox(0,"Error : The file is not read","CasCade Error",MB_ICONERROR);

	SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));
  }

  return aSequence;
}

Standard_Boolean CImportExport::ReadBREP(const Standard_CString& aFileName, TopoDS_Shape& aShape)
{
	BRep_Builder aBuilder;
	Standard_Boolean result = BRepTools::Read(aShape,aFileName,aBuilder);
    return result;
}

//======================================================================

//======================================================================
//=                                                                    =
//=                      STEP                                          =
//=                                                                    =
//======================================================================

bool CImportExport::ReadSTEP(const Handle(AIS_InteractiveContext)& anInteractiveContext)
{
    Handle(TopTools_HSequenceOfShape) aSequence = CImportExport::ReadSTEP();
	
	bool IsImport = false;
	if (!aSequence.IsNull())
	{	
		int length = aSequence->Length();
		for(int i=1;i<= aSequence->Length();i++)
		{
			aisWorkpiece = new AIS_Shape(aSequence->Value(i));
			//m_workpiece.push_back(aisWorkpiece);
			anInteractiveContext->SetDisplayMode( aisWorkpiece , AIS_Shaded);
			anInteractiveContext->Display( aisWorkpiece , Standard_False);
			IsImport = true;
		}
	}
	return IsImport;
}

Handle(TopTools_HSequenceOfShape) CImportExport::ReadSTEP()// not by reference --> the sequence is created here !!
{
  CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
                  "STEP Files (*.stp;*.step)|*.stp; *.step|All Files (*.*)|*.*||", NULL );

  TCHAR tchBuf[80];
  
  Handle(TopTools_HSequenceOfShape) aSequence= new TopTools_HSequenceOfShape();
  if (dlg.DoModal() == IDOK) 
  {
    SetCursor(AfxGetApp()->LoadStandardCursor(IDC_WAIT));
    CString C = dlg.GetPathName();
    Standard_CString aFileName = (Standard_CString)(LPCTSTR)C;
	IFSelect_ReturnStatus ReturnStatus = ReadSTEP(aFileName,aSequence);
	workpiece_name = dlg.GetFileName();
    switch (ReturnStatus) 
    {
       case IFSelect_RetError :
           MessageBox(0,"Not a valid Step file","ERROR",MB_ICONWARNING);
       break;
       case IFSelect_RetFail :
           MessageBox(0,"Reading has failed","ERROR",MB_ICONWARNING);
       break;
       case IFSelect_RetVoid :
            MessageBox(0,"Nothing to transfer","ERROR",MB_ICONWARNING);
       break;
    }
    SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));       
  }
  return aSequence;
}

IFSelect_ReturnStatus CImportExport::ReadSTEP(const Standard_CString& aFileName,
									  std::vector<Handle(AIS_Shape)>& aHSequenceOfAIS_Shape)
{
	//local variable declaration
	std::vector<TopoDS_Shape> compound_shape_vec;
	std::vector<TopoDS_Shape> solid_shape_vec;
	STEPCAFControl_Reader aReader2;
	TDF_LabelSequence step_shapes;
	TDF_LabelSequence step_free_shapes;
	TDF_LabelSequence all_colours;
	std::vector<size_t> assigned_cnt;
	aHSequenceOfAIS_Shape.clear();

	// Create an XCAF Document to contain the STEP file itself
    Handle_TDocStd_Document step_doc;
    static Handle_XCAFApp_Application dummy_app = XCAFApp_Application::GetApplication();

	// Segmentation Faults when trying to create a new document
	if(dummy_app->NbDocuments() > 0)
	{
		dummy_app->GetDocument(1,step_doc);
		dummy_app->Close(step_doc);
	}
	dummy_app->NewDocument ("STEP-XCAF",step_doc);

	//read step file and get read status
	IFSelect_ReturnStatus status = aReader2.ReadFile(aFileName);
	if(IFSelect_RetDone != status){
		return status;
	}

	// Enable transfer of colours
	aReader2.SetColorMode(Standard_True);    
	aReader2.Transfer(step_doc);

	// Read in the shape(s) and the colours present in the STEP File
    Handle_XCAFDoc_ShapeTool step_shape_contents = XCAFDoc_DocumentTool::ShapeTool(step_doc->Main());
    Handle_XCAFDoc_ColorTool step_colour_contents = XCAFDoc_DocumentTool::ColorTool(step_doc->Main());

	// List out the available colors and shapes in the STEP File
	step_shape_contents->GetFreeShapes(step_free_shapes);
	step_shape_contents->GetShapes(step_shapes);
    step_colour_contents->GetColors(all_colours);

	/* for debug*/
	int Length = step_shapes.Length();

	for(int i = 1; i <= step_free_shapes.Length(); i++)
	{
		//Get sub-shapes  
		TDF_LabelSequence step_subshapes;
	    step_shape_contents->GetSubShapes(step_shapes.Value(i),step_subshapes);
		int len_subshapes = step_subshapes.Length();		

		if (step_free_shapes.Value(i).HasChild())
		{
			TDF_ChildIterator it;
			for(it.Initialize(step_free_shapes.Value(i)); it.More(); it.Next())
			{
				//check shape in child Lebel
			    if(step_shape_contents->IsShape(it.Value()))
			    {
					//Get Sub-shape and check it shapetype
					TopoDS_Shape sub_Shape_queue = step_shape_contents->GetShape(it.Value());
				    TopAbs_ShapeEnum temp_shape = sub_Shape_queue.ShapeType();
					switch(sub_Shape_queue.ShapeType())
					{
						case TopAbs_COMPOUND:
							{							
								gp_Trsf Trsf = sub_Shape_queue.Location().Transformation();					

								compound_shape_vec.push_back(sub_Shape_queue);

								TopExp_Explorer Exp;
								for(Exp.Init(sub_Shape_queue,TopAbs_SOLID);Exp.More();Exp.Next())
								{
									TopoDS_Solid temp_Solid = TopoDS::Solid(Exp.Current());
									solid_shape_vec.push_back(temp_Solid);
								}
								
						  }
						  break;
						  case TopAbs_SOLID:
							  TopoDS_Solid temp_Solid = TopoDS::Solid(sub_Shape_queue);
							  solid_shape_vec.push_back(temp_Solid);
						  break;
					}
				}
			}
		}
	}
					
	for(int i = 1; i <= step_shapes.Length(); i++)
	{

		//Get sub-shapes  
		TDF_LabelSequence step_subshapes;
	    step_shape_contents->GetSubShapes(step_shapes.Value(i),step_subshapes);
		int len_subshapes = step_subshapes.Length();		

		//check child Lebel
		if (step_shapes.Value(i).HasChild())
		{
			TDF_ChildIterator it;
			for(it.Initialize(step_shapes.Value(i)); it.More(); it.Next())
			{
				//check shape in child Lebel
			    if(step_shape_contents->IsShape(it.Value()))
			    {
					//Get Sub-shape and check it shapetype
					TopoDS_Shape sub_Shape_queue = step_shape_contents->GetShape(it.Value());
				    TopAbs_ShapeEnum temp_shape = sub_Shape_queue.ShapeType();
					
					//get subshape colors
					TDF_Label l_SubColorLabel;
					Quantity_Color col;
					if(step_colour_contents->GetColor(it.Value(), XCAFDoc_ColorGen, l_SubColorLabel) ||
					   step_colour_contents->GetColor(it.Value(), XCAFDoc_ColorSurf, l_SubColorLabel) ||
					   step_colour_contents->GetColor(it.Value(), XCAFDoc_ColorCurv, l_SubColorLabel) )
					{
						// SubShape has a color ...Quantity_Color &col									
						step_colour_contents->GetColor(l_SubColorLabel,col);									
					}
				    
					switch(sub_Shape_queue.ShapeType())
					{
                        case TopAbs_SOLID:
							{																
							  if(0 == compound_shape_vec.size())
							  {
								  solid_shape_vec.push_back(sub_Shape_queue);								  							  
							  }							  
						  }
						  break;
						case TopAbs_FACE:
							{
								//assign color to AIS_Shape 								
								for(int cnt=0;cnt<solid_shape_vec.size();cnt++)
								{
									TopExp_Explorer Exp;
								    for(Exp.Init(solid_shape_vec[cnt],TopAbs_FACE);Exp.More();Exp.Next())
									{
										
										TopoDS_Face temp_Face = TopoDS::Face(Exp.Current());
									    if(temp_Face.IsPartner(sub_Shape_queue))
									    {
											bool is_assigned = false;
											if(0 == assigned_cnt.size())
											{
												assigned_cnt.push_back(cnt);												
											}
											else
											{
												for(size_t number = 0;number<assigned_cnt.size();number++)
												{
													if(cnt == assigned_cnt[number])
													{
														is_assigned = true;												
														break;
													}													
												}
											}
											if(!is_assigned)
											{
												assigned_cnt.push_back(cnt);

												Handle(AIS_Shape) temp_AIS_Shape =  new AIS_Shape(solid_shape_vec[cnt]);
												temp_AIS_Shape->SetColor(col);

												aHSequenceOfAIS_Shape.push_back(temp_AIS_Shape);
											}
										}
									}							  							  
								}
							}
							break;
					}//end switch(sub_Shape_queue.ShapeType())
				}// end if(step_shape_contents->IsShape(it.Value()))
			}// end for(it.Initialize(step_shapes.Value(i)); it.More(); it.Next())
		}// end if (step_shapes.Value(i).HasChild())
	}// end for(int i = 1; i <= step_shapes.Length(); i++)

	return status;
}

IFSelect_ReturnStatus CImportExport::ReadSTEP(const Standard_CString& aFileName,
                                              Handle(TopTools_HSequenceOfShape)& aHSequenceOfShape
)
{
   aHSequenceOfShape->Clear();

  // create additional log file
  STEPControl_Reader aReader;
  IFSelect_ReturnStatus status = aReader.ReadFile(aFileName);
  if (status != IFSelect_RetDone)
    return status;

  aReader.WS()->MapReader()->SetTraceLevel(2); // increase default trace level

  Standard_Boolean failsonly = Standard_False;
  aReader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity);
  
  // Root transfers
  Standard_Integer nbr = aReader.NbRootsForTransfer();
  aReader.PrintCheckTransfer (failsonly, IFSelect_ItemsByEntity);
  for ( Standard_Integer n = 1; n<=nbr; n++) {
    Standard_Boolean ok = aReader.TransferRoot(n);
  }

  // Collecting resulting entities
  Standard_Integer nbs = aReader.NbShapes();
  if (nbs == 0) {
    return IFSelect_RetVoid;
  }

 
  for (Standard_Integer i=1; i<=nbs; i++) {
    aHSequenceOfShape->Append(aReader.Shape(i));	
  }

  return status;
}
