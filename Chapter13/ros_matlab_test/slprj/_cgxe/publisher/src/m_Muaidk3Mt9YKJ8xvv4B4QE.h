#ifndef __Muaidk3Mt9YKJ8xvv4B4QE_h__
#define __Muaidk3Mt9YKJ8xvv4B4QE_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"

/* Type Definitions */
#ifndef enum_robotics_core_internal_OverrunActions
#define enum_robotics_core_internal_OverrunActions

enum robotics_core_internal_OverrunActions
{
  robotics_core_internal_OverrunActions_Drop = 1,/* Default value */
  robotics_core_internal_OverrunActions_Slip
};

#endif                                 /*enum_robotics_core_internal_OverrunActions*/

#ifndef typedef_robotics_core_internal_OverrunActions
#define typedef_robotics_core_internal_OverrunActions

typedef enum robotics_core_internal_OverrunActions
  robotics_core_internal_OverrunActions;

#endif                                 /*typedef_robotics_core_internal_OverrunActions*/

#ifndef typedef_robotics_core_internal_SystemTimeProvider
#define typedef_robotics_core_internal_SystemTimeProvider

typedef struct {
  real_T StartTime;
} robotics_core_internal_SystemTimeProvider;

#endif                                 /*typedef_robotics_core_internal_SystemTimeProvider*/

#ifndef typedef_b_robotics_core_internal_OverrunActions
#define typedef_b_robotics_core_internal_OverrunActions

typedef int32_T b_robotics_core_internal_OverrunActions;

#endif                                 /*typedef_b_robotics_core_internal_OverrunActions*/

#ifndef b_robotics_core_internal_OverrunActions_constants
#define b_robotics_core_internal_OverrunActions_constants

/* enum b_robotics_core_internal_OverrunActions */
#define robotics_core_internal_OverrunActions_Drop (1)
#define robotics_core_internal_OverrunActions_Slip (2)
#endif                                 /*b_robotics_core_internal_OverrunActions_constants*/

#ifndef typedef_robotics_Rate
#define typedef_robotics_Rate

typedef struct {
  real_T DesiredRate;
  real_T DesiredPeriod;
  real_T LastWakeTime;
  robotics_core_internal_SystemTimeProvider TimeProvider;
  b_robotics_core_internal_OverrunActions InternalOverrunAction;
  real_T PeriodCount;
  real_T NumOverruns;
  real_T NextExecutionIndex;
  real_T ExecutionStartTime;
} robotics_Rate;

#endif                                 /*typedef_robotics_Rate*/

#ifndef typedef_ros_rate
#define typedef_ros_rate

typedef struct {
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T RATE;
  robotics_Rate rateObj;
} ros_rate;

#endif                                 /*typedef_ros_rate*/

#ifndef typedef_InstanceStruct_Muaidk3Mt9YKJ8xvv4B4QE
#define typedef_InstanceStruct_Muaidk3Mt9YKJ8xvv4B4QE

typedef struct {
  SimStruct *S;
  ros_rate sysobj;
  boolean_T sysobj_not_empty;
  void *emlrtRootTLSGlobal;
  covrtInstance *covInst_CONTAINER_BLOCK_INDEX;
} InstanceStruct_Muaidk3Mt9YKJ8xvv4B4QE;

#endif                                 /*typedef_InstanceStruct_Muaidk3Mt9YKJ8xvv4B4QE*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_Muaidk3Mt9YKJ8xvv4B4QE(SimStruct *S, int_T method,
  void* data);

#endif
