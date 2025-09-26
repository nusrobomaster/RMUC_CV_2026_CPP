#include "util.h"

namespace calibur {
pid_t GetThreadId() {
    return gettid();
}

uint32_t GetFiberId() {
    return 0;
}
}