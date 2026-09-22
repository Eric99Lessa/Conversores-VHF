/* Include files */

#include "boost_control_cgxe.h"
#include "m_OmrnQES9qogeQTNCDGc0yB.h"

unsigned int cgxe_boost_control_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 4174618450 &&
      ssGetChecksum1(S) == 1453203943 &&
      ssGetChecksum2(S) == 2128399603 &&
      ssGetChecksum3(S) == 4238852223) {
    method_dispatcher_OmrnQES9qogeQTNCDGc0yB(S, method, data);
    return 1;
  }

  return 0;
}
