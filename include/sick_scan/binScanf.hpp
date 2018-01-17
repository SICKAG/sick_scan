#ifndef BINSCANF_HPP
#define BINSCANF_HPP

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <vector>
int
binScanf(const char *fmt, ...);
int
binScanfVec(const std::vector<unsigned char> *vec, const char *fmt, ...);
#endif
