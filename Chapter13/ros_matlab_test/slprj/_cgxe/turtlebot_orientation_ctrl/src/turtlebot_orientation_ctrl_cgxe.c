/* Include files */

#include "turtlebot_orientation_ctrl_cgxe.h"
#include "m_xkq2bYMqTPePvRauP0qkrG.h"

unsigned int cgxe_turtlebot_orientation_ctrl_method_dispatcher(SimStruct* S,
  int_T method, void* data)
{
  if (ssGetChecksum0(S) == 4018213974 &&
      ssGetChecksum1(S) == 2811207502 &&
      ssGetChecksum2(S) == 1664727671 &&
      ssGetChecksum3(S) == 449789046) {
    method_dispatcher_xkq2bYMqTPePvRauP0qkrG(S, method, data);
    return 1;
  }

  return 0;
}
