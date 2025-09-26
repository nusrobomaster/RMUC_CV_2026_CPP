#ifndef __CALIBUR_UTIL_H__
#define __CALIBUR_UTIL_H__

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

namespace calibur {
pid_t GetThreadId();

uint32_t GetFiberId();
}

#endif