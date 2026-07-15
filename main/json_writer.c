#include "json_writer.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static bool json_writer_ready(const json_writer_t *writer)
{
    return writer && writer->valid && writer->buffer && writer->capacity > 0 &&
           writer->length < writer->capacity;
}

static bool json_writer_fail(json_writer_t *writer)
{
    if (writer) {
        writer->valid = false;
        if (writer->buffer && writer->capacity > 0) {
            size_t terminator = writer->length < writer->capacity ? writer->length : 0;
            writer->buffer[terminator] = '\0';
        }
    }
    return false;
}

static bool json_writer_append_bytes(json_writer_t *writer, const char *data, size_t size)
{
    if (!json_writer_ready(writer) || (!data && size > 0)) {
        return json_writer_fail(writer);
    }

    size_t remaining = writer->capacity - writer->length;
    if (size >= remaining) {
        return json_writer_fail(writer);
    }

    if (size > 0) {
        memcpy(writer->buffer + writer->length, data, size);
        writer->length += size;
    }
    writer->buffer[writer->length] = '\0';
    return true;
}

void json_writer_init(json_writer_t *writer, char *buffer, size_t capacity)
{
    if (!writer) {
        return;
    }

    writer->buffer = buffer;
    writer->capacity = capacity;
    writer->length = 0;
    writer->valid = buffer != NULL && capacity > 0;
    if (writer->valid) {
        buffer[0] = '\0';
    }
}

bool json_writer_appendf(json_writer_t *writer, const char *format, ...)
{
    if (!json_writer_ready(writer) || !format) {
        return json_writer_fail(writer);
    }

    size_t start = writer->length;
    size_t remaining = writer->capacity - start;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(writer->buffer + start, remaining, format, args);
    va_end(args);

    if (written < 0 || (size_t)written >= remaining) {
        writer->buffer[start] = '\0';
        return json_writer_fail(writer);
    }

    writer->length = start + (size_t)written;
    return true;
}

bool json_writer_append_escaped_string(json_writer_t *writer, const char *value)
{
    if (!json_writer_ready(writer)) {
        return json_writer_fail(writer);
    }
    if (!value) {
        return json_writer_append_bytes(writer, "null", 4);
    }

    size_t value_length = strlen(value);
    size_t encoded_size = 2; /* Opening and closing quotes. */
    for (size_t i = 0; i < value_length; ++i) {
        unsigned char byte = (unsigned char)value[i];
        size_t added = 1;
        switch (byte) {
        case '"':
        case '\\':
        case '\b':
        case '\f':
        case '\n':
        case '\r':
        case '\t':
            added = 2;
            break;
        default:
            if (byte < 0x20) {
                added = 6; /* \u00XX */
            }
            break;
        }
        if (encoded_size > SIZE_MAX - added) {
            return json_writer_fail(writer);
        }
        encoded_size += added;
    }

    size_t remaining = writer->capacity - writer->length;
    if (encoded_size >= remaining) {
        return json_writer_fail(writer);
    }

    static const char hex[] = "0123456789abcdef";
    char *out = writer->buffer + writer->length;
    *out++ = '"';
    for (size_t i = 0; i < value_length; ++i) {
        unsigned char byte = (unsigned char)value[i];
        switch (byte) {
        case '"':
            *out++ = '\\';
            *out++ = '"';
            break;
        case '\\':
            *out++ = '\\';
            *out++ = '\\';
            break;
        case '\b':
            *out++ = '\\';
            *out++ = 'b';
            break;
        case '\f':
            *out++ = '\\';
            *out++ = 'f';
            break;
        case '\n':
            *out++ = '\\';
            *out++ = 'n';
            break;
        case '\r':
            *out++ = '\\';
            *out++ = 'r';
            break;
        case '\t':
            *out++ = '\\';
            *out++ = 't';
            break;
        default:
            if (byte < 0x20) {
                *out++ = '\\';
                *out++ = 'u';
                *out++ = '0';
                *out++ = '0';
                *out++ = hex[byte >> 4];
                *out++ = hex[byte & 0x0f];
            } else {
                *out++ = (char)byte;
            }
            break;
        }
    }
    *out++ = '"';
    *out = '\0';
    writer->length += encoded_size;
    return true;
}

bool json_writer_ok(const json_writer_t *writer)
{
    return json_writer_ready(writer);
}

size_t json_writer_length(const json_writer_t *writer)
{
    return writer ? writer->length : 0;
}
