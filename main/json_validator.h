#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Validate one complete RFC 8259 JSON object without allocating memory. */
bool json_validate_object(const char *text, size_t length);

#ifdef __cplusplus
}
#endif
