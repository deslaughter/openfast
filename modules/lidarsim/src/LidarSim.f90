    MODULE LidarSim

    USE LidarSim_Types
    USE LidarSim_Subs
    USE NWTC_Library
    USE InflowWind
    USE InflowWind_Subs
    USE InflowWind_Types

    IMPLICIT NONE
    PRIVATE

    TYPE(ProgDesc), PARAMETER   ::  IfW_Ver = ProgDesc( 'LidarSim', 'v0.20', '12-December-2019' )
    PUBLIC                      ::  LidarSim_Init                                
    PUBLIC                      ::  LidarSim_CalcOutput    
    PUBLIC                      ::  LidarSim_End

    CONTAINS

    !#########################################################################################################################################################################

SUBROUTINE LidarSim_Init(InitInp, y, p, InitOutData, ErrStat, ErrMsg )

    IMPLICIT                                NONE
    CHARACTER(*),                           PARAMETER       ::  RoutineName="LidarSim_Init"
    
    TYPE(LidarSim_InitInputType),           INTENT(IN   )   ::  InitInp             ! Input data for initialization routine
    TYPE(LidarSim_OutputType),              INTENT(  OUT)   ::  y                   ! Output data for the lidar module
    TYPE(LidarSim_ParameterType),           INTENT(  OUT)   ::  p                   ! Parameter data for the lidar module
    TYPE(LidarSim_InitOutputType),          INTENT(  OUT)   ::  InitOutData         ! Data to initialize the outputs
    INTEGER(IntKi),                         INTENT(  OUT)   ::  ErrStat             !< Error status of the operation
    CHARACTER(*),                           INTENT(  OUT)   ::  ErrMsg              !< Error message if ErrStat /= ErrID_None

    !Local Variables
    TYPE(LidarSim_InputFile)                                ::  InputFileData      !< Structure to load the input file data into
    CHARACTER(1024)                                         ::  RootFileName
    CHARACTER(1024)                                         ::  EchoFileName

    ! Temporary variables for error handling
    INTEGER(IntKi)                                          ::  TmpErrStat          !< temporary error message
    CHARACTER(ErrMsgLen)                                    ::  TmpErrMsg           
    
    
    ! Initial error values
    ErrStat        =  0
    ErrMsg         =  ""  
    
    RootFileName  = InitInp%RootName
    IF (LEN_TRIM(RootFileName) == 0) CALL GetRoot( InitInp%InputInitFile, RootFileName )
    EchoFileName  = TRIM(RootFileName)//".ech"

    
    ! Reads the Config from the Input file and writes it into the LidarSim_InputFile data 
    CALL LidarSim_ReadInputFile(InitInp%InputInitFile,EchoFileName , InputFileData, TmpErrStat, TmpErrMsg)!EchoFileName
      if (Failed()) return

    
    !Transfering InputFileData to the p
    p%MeasurementMaxSteps   =   CEILING(REAL(NINT(InputFileData%t_measurement_interval*100000))/REAL(NINT(InitInp%DT*100000))) !NINT to remove float precision errors. Back to REAL, otherwise the divion ignores everything behind the decima point. Ceiling to round up to next integer
    p%LidarPosition_N(1)    =   InputFileData%LidarPositionX_N
    p%LidarPosition_N(2)    =   InputFileData%LidarPositionY_N
    p%LidarPosition_N(3)    =   InputFileData%LidarPositionZ_N
    p%URef                  =   InputFileData%URef
    p%GatesPerBeam          =   InputFileData%GatesPerBeam
    p%MAXDLLChainOutputs    =   InputFileData%MAXDLLChainOutputs
 
    !Creates the static rotationmatrix from the lidar system to the nacelle system
    CALL LidarSim_CreateRotationMatrix(InputFileData%RollAngle_N,InputFileData%PitchAngle_N,&
    InputFileData%YawAngle_N, p%LidarOrientation_N)

    !----- Calls different Subroutines to initialize the measuring points   
    IF(InputFileData%TrajectoryType == 0) THEN
        CALL LidarSim_InitMeasuringPoints_Cartesian(p, InputFileData, TmpErrStat, TmpErrMsg)    ! Calls Routine to initialize cartesian coordinate inputs
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
    ELSEIF(InputFileData%TrajectoryType == 1)THEN
        CALL LidarSim_InitMeasuringPoints_Spherical(p, InputFileData, TmpErrStat, TmpErrMsg )   ! Calls Routine to initialize spherical coordinate inputs
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
    END IF
    
    !----- Calls different Subroutines to initialize the weighting points
    IF(InputFileData%WeightingType == 0) THEN                                                   ! Single point
        CALL AllocAry( p%WeightingDistance,1, 'p%WeightingDistance', TmpErrStat, TmpErrMsg )
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
        CALL AllocAry( p%Weighting,1, 'p%Weighting', TmpErrStat, TmpErrMsg )
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
        p%WeightingDistance(1) = 0
        p%Weighting(1) = 1
    ELSEIF(InputFileData%WeightingType == 1) THEN                                               ! Calls Routine to initialize the weighting with gaussian distribution
        CALL LidarSim_InitializeWeightingGauss(p, InputFileData, TmpErrStat, TmpErrMsg )       
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
    ELSEIF(InputFileData%WeightingType == 2) THEN
        CALL LidarSim_InitializeWeightingManual(p, InputFileData, TmpErrStat, TmpErrMsg )       ! Calls Routine to initialize with manual weighting settings
        CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
    ENDIF

    CALL LidarSim_InitializeOutputs(y,p, InitOutData, InputFileData, TmpErrStat, TmpErrMsg )
    CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
    
    !initialize variables and outputs
    p%MeasurementCurrentStep = -1                                                               !< there was no measurement yet
    p%LastMeasuringPoint = 1                                                                    !< First measurement point
    p%NextBeamID = 0
    
   call Cleanup()
   return
 
   contains
   logical function Failed()
      CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
      Failed = ErrStat >= AbortErrLev
      if (Failed) then
         call Cleanup
      endif
   end function Failed

   subroutine Cleanup()
      CALL LidarSim_DestroyInputFile(InputFileData, TmpErrStat, TmpErrMsg)                              ! Calls to destory the data from the inputfile. Important data has to be transfered to the parameter data before
      CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)
   end subroutine Cleanup

END SUBROUTINE LidarSim_Init

    !#########################################################################################################################################################################

SUBROUTINE LidarSim_CalcOutput (Time, y, p, u,&
    IfW_p, IfW_ContStates, IfW_DiscStates, IfW_ConstrStates, IfW_OtherStates,  IfW_m,&
    ErrStat, ErrMsg )

    IMPLICIT                                    NONE
    CHARACTER(*),                               PARAMETER           ::  RoutineName="LidarSim_CalcOutput"

    REAL(DbKi),                                 INTENT(IN   )       ::  Time                !< Current simulation time in seconds
    TYPE(LidarSim_ParameterType),               INTENT(INOUT)       ::  p
    TYPE(LidarSim_OutputType),                  INTENT(INOUT)       ::  y                   !< Outputs computed at Time (IN for mesh reasons and data allocation)
    TYPE(LidarSim_InputType),                   INTENT(IN   )       ::  u                   !< Inputs from other Modules (e.g. ElastoDyn)

    !Data for CalcOutput of IfW_Subs
    TYPE(InflowWind_ParameterType),             INTENT(IN   )       ::  IfW_p                       !< Parameters
    TYPE(InflowWind_ContinuousStateType),       INTENT(IN   )       ::  IfW_ContStates              !< Continuous states at Time
    TYPE(InflowWind_DiscreteStateType),         INTENT(IN   )       ::  IfW_DiscStates              !< Discrete states at Time
    TYPE(InflowWind_ConstraintStateType),       INTENT(IN   )       ::  IfW_ConstrStates            !< Constraint states at Time
    TYPE(InflowWind_OtherStateType),            INTENT(IN   )       ::  IfW_OtherStates             !< Other/optimization states at Time
    TYPE(InflowWind_MiscVarType),               INTENT(INOUT)       ::  IfW_m                       !< Misc variables for optimization (not copied in glue code)    
    INTEGER(IntKi),                             INTENT(  OUT)       ::  ErrStat                     !< Error status of the operation
    CHARACTER(*),                               INTENT(  OUT)       ::  ErrMsg                      !< Error message if ErrStat /= ErrID_None

    !Local Variables
    TYPE(InflowWind_InputType)                                      ::  InputForCalculation         !Data Field needed for the calculation of the windspeed
    TYPE(InflowWind_OutputType)                                     ::  OutputForCalculation        !datafield in which the calculated speed is stored
    REAL(ReKi)                                                      ::  UnitVector(3)               !Line of Sight Unit Vector
    REAL(ReKi)                                                      ::  MeasuringPosition_I(3)      !Transformed Measuring Position
    REAL(ReKi)                                                      ::  LidarPosition_I(3)          !Transformed Lidar Position
    REAL(ReKi)                                                      ::  Vlos                        !Line of sight speed
    INTEGER(IntKi)                                                  ::  LoopGatesPerBeam            !Counter to loop through all gate points of a line
    INTEGER(IntKi)                                                  ::  LoopCounter

    ! Temporary variables for error handling
    INTEGER(IntKi)                                                  ::  TmpErrStat        
    CHARACTER(ErrMsgLen)                                            ::  TmpErrMsg
    
    !Initialize error values
    ErrStat        =  0
    ErrMsg         =  ""

    
    IF(p%MeasurementCurrentStep>=p%MeasurementMaxSteps .OR. p%MeasurementCurrentStep == -1)THEN         !Check if there must be a new measurement     !(NINT(Time*1000)-NINT(p%t_last_measurement*1000)) >= NINT(p%t_measurement_interval*1000)
        p%MeasurementCurrentStep = 0
        LidarPosition_I = LidarSim_TransformLidarToInertial( u%NacelleMotion,p, (/0.0,0.0,0.0/) )                               !Calculation of the lidar positon ( 0 / 0 / 0 ) in the lidar coordinate system

        CALL LidarSim_CalculateIMU(p, y, u)
        DO LoopGatesPerBeam = 0,p%GatesPerBeam-1
            !Transform measuring and lidar position to the inertial system
            MeasuringPosition_I = LidarSim_TransformLidarToInertial(u%NacelleMotion,p,p%MeasuringPoints_L(:,p%LastMeasuringPoint+LoopGatesPerBeam)) ! Calculate the Measuringpoint coordinate in the initial system

            !Line of Sight
            UnitVector    =   MeasuringPosition_I - LidarPosition_I             !Calculation of the Line of Sight Vector          
            UnitVector    =   UnitVector/NORM2(UnitVector)                      !=>Magnitude = 1

            !Calculation of the wind speed at the calculated position
            CALL LidarSim_CalculateVlos( p, UnitVector, Vlos, MeasuringPosition_I, LidarPosition_I, Time, IfW_p, IfW_ContStates, IfW_DiscStates, IfW_ConstrStates, IfW_OtherStates, IfW_m, TmpErrStat, TmpErrMsg) !Calculation of the line of sight wind speed
            CALL SetErrStat(TmpErrStat,TmpErrMsg,ErrStat,ErrMsg,RoutineName)    
            CALL LidarSim_SetOutputs(y,p,Vlos,UnitVector,LoopGatesPerBeam,Time)    !Set all outputs to the output variable
        ENDDO                
        
        !Choosing which measuring point has to be calculated
        IF(p%LastMeasuringPoint+p%GatesPerBeam > SIZE(p%MeasuringPoints_L,2))THEN                      
            p%LastMeasuringPoint = 1        ! already reached the last point before ? => start over from the beginning
            p%NextBeamID = 0
        ELSE
            p%LastMeasuringPoint = p%LastMeasuringPoint + p%GatesPerBeam
            p%NextBeamID = p%NextBeamID + 1
        END IF
    ELSE                                            !Set NewData signals to zero
!FIXME: move ValidOutputs out of p
        IF(ANY(p%ValidOutputs == 22)) THEN
            DO LoopCounter = 1,SIZE(p%ValidOutputs)
                IF(p%ValidOutputs(LoopCounter) == 22) THEN
                    y%WriteOutput(LoopCounter) = 0
                END IF
            END DO
        END IF
        y%SwapOutputs(1) = 0
    ENDIF
    p%MeasurementCurrentStep = p%MeasurementCurrentStep + 1

    END SUBROUTINE LidarSim_CalcOutput

    !#########################################################################################################################################################################

    SUBROUTINE LidarSim_End( y, p, u, ErrStat, ErrMsg)

    IMPLICIT                                 NONE    
    CHARACTER(*),                            PARAMETER       :: RoutineName="LidarSim_End"

    TYPE(LidarSim_InputType),                INTENT(INOUT)   ::  u           !< Input data for initialization
    TYPE(LidarSim_ParameterType),            INTENT(INOUT)   ::  p           !< Parameters
    TYPE(LidarSim_OutputType),               INTENT(INOUT)   ::  y           !< Output data
    
    ! Error Handling
    INTEGER( IntKi ),                        INTENT(  OUT)   :: ErrStat      !< error status
    CHARACTER(*),                            INTENT(  OUT)   :: ErrMsg       !< error message
    
    
    ErrStat = ErrID_None
    ErrMsg = ""
    
    CALL LidarSim_DestroyOutput(y, ErrStat, ErrMsg)
    CALL LidarSim_DestroyParam(p, ErrStat, ErrMsg)

    END SUBROUTINE LidarSim_End
    
    !#########################################################################################################################################################################
    
    END MODULE LidarSim
