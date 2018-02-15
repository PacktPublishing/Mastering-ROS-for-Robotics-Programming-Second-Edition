/* Include files */

#include "publisher_cgxe.h"
#include "m_Muaidk3Mt9YKJ8xvv4B4QE.h"

unsigned int cgxe_publisher_method_dispatcher(SimStruct* S, int_T method, void
  * data)
{
  if (ssGetChecksum0(S) == 1367728901 &&
      ssGetChecksum1(S) == 1484596684 &&
      ssGetChecksum2(S) == 3577695903 &&
      ssGetChecksum3(S) == 643404390) {
    method_dispatcher_Muaidk3Mt9YKJ8xvv4B4QE(S, method, data);
    return 1;
  }

  return 0;
}
