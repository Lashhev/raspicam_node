#ifdef __x86_64__

#include <stdio.h>

int main(int argc, char** argv) {
  (void)fprintf(stderr, "The raspicam_node for the x86/64 architecture is a fake!\n");
  return 1;
}

#endif  // __x86_64__

#ifdef __arm__
#include "mmal_cxx_helper.h"
#include "interfaces/raspicam_interface.hpp"

int main(int argc, char* argv[])
{
    RASPIVID_STATE state;
    printf("%p\n", std::move(state.splitter_pool));
    return 0;
}
#endif  // __arm__