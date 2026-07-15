#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fixed-buffer JSON writer.
 *
 * The caller owns the buffer. A writer never allocates memory and keeps every
 * successfully initialized, non-empty buffer NUL terminated. Once an append
 * fails, the failure is latched and later appends are ignored.
 */
typedef struct {
    char *buffer;
    size_t capacity;
    size_t length;
    bool valid;
} json_writer_t;

/**
 * @brief Initialize a writer over a caller-provided buffer.
 *
 * A NULL buffer or zero capacity creates a failed writer. Otherwise the buffer
 * is reset to an empty, NUL-terminated string.
 */
void json_writer_init(json_writer_t *writer, char *buffer, size_t capacity);

/**
 * @brief Append formatted text.
 *
 * This is intended for trusted JSON syntax and scalar formatting. If
 * vsnprintf reports an error or truncation, the partial append is discarded,
 * the previous complete content is retained, and the writer is marked failed.
 * Use json_writer_append_escaped_string() for untrusted string values.
 */
#if defined(__GNUC__) || defined(__clang__)
bool json_writer_appendf(json_writer_t *writer, const char *format, ...)
    __attribute__((format(printf, 2, 3)));
#else
bool json_writer_appendf(json_writer_t *writer, const char *format, ...);
#endif

/**
 * @brief Append one complete JSON string value, including its quotes.
 *
 * Quotes, backslashes, and JSON control characters are escaped. Other bytes
 * are copied unchanged so UTF-8 input is preserved. A NULL value is encoded as
 * the JSON literal null. Capacity is checked before writing, so failure leaves
 * the previous complete content unchanged.
 */
bool json_writer_append_escaped_string(json_writer_t *writer, const char *value);

/** @brief Return true while no initialization or append failure has occurred. */
bool json_writer_ok(const json_writer_t *writer);

/** @brief Return the number of complete bytes written, excluding the NUL. */
size_t json_writer_length(const json_writer_t *writer);

#ifdef __cplusplus
}
#endif
