/* Include files */

#include "boost_control_cgxe.h"
#include "m_6xrqX5qITXgQNjIsHQxKhB.h"
#include "m_Vk25VrZsrlIfY8DkqnY5WG.h"

unsigned int cgxe_boost_control_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 2195679973 &&
      ssGetChecksum1(S) == 2138090989 &&
      ssGetChecksum2(S) == 477986389 &&
      ssGetChecksum3(S) == 811909483) {
    method_dispatcher_6xrqX5qITXgQNjIsHQxKhB(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 4158152884 &&
      ssGetChecksum1(S) == 1889023061 &&
      ssGetChecksum2(S) == 288648657 &&
      ssGetChecksum3(S) == 2235324607) {
    method_dispatcher_Vk25VrZsrlIfY8DkqnY5WG(S, method, data);
    return 1;
  }

  return 0;
}
