//!STARTOFREGISTRYGENERATEDFILE 'ExternalInflow_Types.h'
//!
//! WARNING This file is generated automatically by the FAST registry.
//! Do not edit.  Your changes to this file will be lost.
//!

#ifndef _ExternalInflow_TYPES_H
#define _ExternalInflow_TYPES_H


#ifdef _WIN32 //define something for Windows (32-bit)
#  include "stdbool.h"
#  define CALL __declspec( dllexport )
#elif _WIN64 //define something for Windows (64-bit)
#  include "stdbool.h"
#  define CALL __declspec( dllexport ) 
#else
#  include <stdbool.h>
#  define CALL 
#endif


  typedef struct ExtInfw_InitInputType {
    void * object ;
    int NumActForcePtsBlade ;
    int NumActForcePtsTower ;
    int * nStructBldEtaNodes ;     int nStructBldEtaNodes_Len ;
    double * StructBldEtaNodes ;     int StructBldEtaNodes_Len ;
    double * StructTwrEtaNodes ;     int StructTwrEtaNodes_Len ;
    int NumBl ;
    float BladeLength ;
    float TowerHeight ;
    float TowerBaseHeight ;
  } ExtInfw_InitInputType_t ;
  typedef struct ExtInfw_InitOutputType {
    void * object ;
    char * WriteOutputHdr ;     int WriteOutputHdr_Len ;
    char * WriteOutputUnt ;     int WriteOutputUnt_Len ;

  } ExtInfw_InitOutputType_t ;
  typedef struct ExtInfw_MiscVarType {
    void * object ;








  } ExtInfw_MiscVarType_t ;
  typedef struct ExtInfw_ParameterType {
    void * object ;
    float AirDens ;
    int NumBl ;
    int NMappings ;
    int NnodesVel ;
    int NnodesForce ;
    int NnodesForceBlade ;
    int NnodesForceTower ;
    float * forceBldEtaNodes ;     int forceBldEtaNodes_Len ;
    float * forceTwrEtaNodes ;     int forceTwrEtaNodes_Len ;
  } ExtInfw_ParameterType_t ;
  typedef struct ExtInfw_InputType {
    void * object ;
    float * pxVel ;     int pxVel_Len ;
    float * pyVel ;     int pyVel_Len ;
    float * pzVel ;     int pzVel_Len ;
    float * pxdotVel ;     int pxdotVel_Len ;
    float * pydotVel ;     int pydotVel_Len ;
    float * pzdotVel ;     int pzdotVel_Len ;
    float * pxForce ;     int pxForce_Len ;
    float * pyForce ;     int pyForce_Len ;
    float * pzForce ;     int pzForce_Len ;
    float * pxdotForce ;     int pxdotForce_Len ;
    float * pydotForce ;     int pydotForce_Len ;
    float * pzdotForce ;     int pzdotForce_Len ;
    float * pOrientation ;     int pOrientation_Len ;
    float * fx ;     int fx_Len ;
    float * fy ;     int fy_Len ;
    float * fz ;     int fz_Len ;
    float * momentx ;     int momentx_Len ;
    float * momenty ;     int momenty_Len ;
    float * momentz ;     int momentz_Len ;
    float * forceRHloc ;     int forceRHloc_Len ;
    float * forceNodesChord ;     int forceNodesChord_Len ;
  } ExtInfw_InputType_t ;
  typedef struct ExtInfw_OutputType {
    void * object ;
    float * u ;     int u_Len ;
    float * v ;     int v_Len ;
    float * w ;     int w_Len ;
    float * WriteOutput ;     int WriteOutput_Len ;
  } ExtInfw_OutputType_t ;
  typedef struct ExtInfw_UserData {
    ExtInfw_InitInputType_t        ExtInfw_InitInput ;
    ExtInfw_InitOutputType_t       ExtInfw_InitOutput ;
    ExtInfw_MiscVarType_t          ExtInfw_Misc ;
    ExtInfw_ParameterType_t        ExtInfw_Param ;
    ExtInfw_InputType_t            ExtInfw_Input ;
    ExtInfw_OutputType_t           ExtInfw_Output ;
  } ExtInfw_t ;

#endif // _ExternalInflow_TYPES_H


//!ENDOFREGISTRYGENERATEDFILE
