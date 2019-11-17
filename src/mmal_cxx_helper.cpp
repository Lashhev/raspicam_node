
#include "mmal_cxx_helper.h"

namespace mmal
{
  void default_delete_pool(MMAL_POOL_T* ptr) {
    if (ptr != nullptr) {
      fprintf(stderr, "%s\n", "LEAKED POOL! you need to define your own deleter");
    }
  }
}