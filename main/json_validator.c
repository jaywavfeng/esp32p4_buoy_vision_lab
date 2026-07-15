#include "json_validator.h"

#include <stdint.h>

#define JSON_VALIDATOR_MAX_DEPTH 32U

typedef struct {
    const char *cursor;
    const char *end;
} json_parser_t;

static void skip_whitespace(json_parser_t *parser)
{
    while (parser->cursor < parser->end) {
        char ch = *parser->cursor;
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            break;
        }
        parser->cursor++;
    }
}

static bool is_hex(char ch)
{
    return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') ||
           (ch >= 'A' && ch <= 'F');
}

static bool consume_utf8_tail(json_parser_t *parser, unsigned char first)
{
    size_t tail_bytes = 0;
    unsigned char second_min = 0x80U;
    unsigned char second_max = 0xBFU;
    if (first >= 0xC2U && first <= 0xDFU) {
        tail_bytes = 1;
    } else if (first >= 0xE0U && first <= 0xEFU) {
        tail_bytes = 2;
        if (first == 0xE0U) {
            second_min = 0xA0U;
        } else if (first == 0xEDU) {
            second_max = 0x9FU;
        }
    } else if (first >= 0xF0U && first <= 0xF4U) {
        tail_bytes = 3;
        if (first == 0xF0U) {
            second_min = 0x90U;
        } else if (first == 0xF4U) {
            second_max = 0x8FU;
        }
    } else {
        return false;
    }
    if ((size_t)(parser->end - parser->cursor) < tail_bytes) {
        return false;
    }
    unsigned char second = (unsigned char)parser->cursor[0];
    if (second < second_min || second > second_max) {
        return false;
    }
    for (size_t i = 1; i < tail_bytes; i++) {
        unsigned char byte = (unsigned char)parser->cursor[i];
        if (byte < 0x80U || byte > 0xBFU) {
            return false;
        }
    }
    parser->cursor += tail_bytes;
    return true;
}

static bool parse_string(json_parser_t *parser)
{
    if (parser->cursor >= parser->end || *parser->cursor++ != '"') {
        return false;
    }
    while (parser->cursor < parser->end) {
        unsigned char ch = (unsigned char)*parser->cursor++;
        if (ch == '"') {
            return true;
        }
        if (ch < 0x20U) {
            return false;
        }
        if (ch != '\\') {
            if (ch >= 0x80U && !consume_utf8_tail(parser, ch)) {
                return false;
            }
            continue;
        }
        if (parser->cursor >= parser->end) {
            return false;
        }
        ch = (unsigned char)*parser->cursor++;
        if (ch == '"' || ch == '\\' || ch == '/' || ch == 'b' || ch == 'f' ||
            ch == 'n' || ch == 'r' || ch == 't') {
            continue;
        }
        if (ch != 'u' || (size_t)(parser->end - parser->cursor) < 4U) {
            return false;
        }
        for (size_t i = 0; i < 4U; i++) {
            if (!is_hex(parser->cursor[i])) {
                return false;
            }
        }
        parser->cursor += 4;
    }
    return false;
}

static bool parse_number(json_parser_t *parser)
{
    const char *start = parser->cursor;
    if (parser->cursor < parser->end && *parser->cursor == '-') {
        parser->cursor++;
    }
    if (parser->cursor >= parser->end) {
        return false;
    }
    if (*parser->cursor == '0') {
        parser->cursor++;
        if (parser->cursor < parser->end && *parser->cursor >= '0' &&
            *parser->cursor <= '9') {
            return false;
        }
    } else if (*parser->cursor >= '1' && *parser->cursor <= '9') {
        do {
            parser->cursor++;
        } while (parser->cursor < parser->end && *parser->cursor >= '0' &&
                 *parser->cursor <= '9');
    } else {
        return false;
    }
    if (parser->cursor < parser->end && *parser->cursor == '.') {
        parser->cursor++;
        if (parser->cursor >= parser->end || *parser->cursor < '0' ||
            *parser->cursor > '9') {
            return false;
        }
        do {
            parser->cursor++;
        } while (parser->cursor < parser->end && *parser->cursor >= '0' &&
                 *parser->cursor <= '9');
    }
    if (parser->cursor < parser->end &&
        (*parser->cursor == 'e' || *parser->cursor == 'E')) {
        parser->cursor++;
        if (parser->cursor < parser->end &&
            (*parser->cursor == '+' || *parser->cursor == '-')) {
            parser->cursor++;
        }
        if (parser->cursor >= parser->end || *parser->cursor < '0' ||
            *parser->cursor > '9') {
            return false;
        }
        do {
            parser->cursor++;
        } while (parser->cursor < parser->end && *parser->cursor >= '0' &&
                 *parser->cursor <= '9');
    }
    return parser->cursor > start;
}

static bool consume_literal(json_parser_t *parser, const char *literal, size_t length)
{
    if ((size_t)(parser->end - parser->cursor) < length) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        if (parser->cursor[i] != literal[i]) {
            return false;
        }
    }
    parser->cursor += length;
    return true;
}

static bool parse_value(json_parser_t *parser, uint32_t depth);

static bool parse_array(json_parser_t *parser, uint32_t depth)
{
    if (depth >= JSON_VALIDATOR_MAX_DEPTH || parser->cursor >= parser->end ||
        *parser->cursor++ != '[') {
        return false;
    }
    skip_whitespace(parser);
    if (parser->cursor < parser->end && *parser->cursor == ']') {
        parser->cursor++;
        return true;
    }
    while (parse_value(parser, depth + 1U)) {
        skip_whitespace(parser);
        if (parser->cursor >= parser->end) {
            return false;
        }
        char separator = *parser->cursor++;
        if (separator == ']') {
            return true;
        }
        if (separator != ',') {
            return false;
        }
        skip_whitespace(parser);
    }
    return false;
}

static bool parse_object(json_parser_t *parser, uint32_t depth)
{
    if (depth >= JSON_VALIDATOR_MAX_DEPTH || parser->cursor >= parser->end ||
        *parser->cursor++ != '{') {
        return false;
    }
    skip_whitespace(parser);
    if (parser->cursor < parser->end && *parser->cursor == '}') {
        parser->cursor++;
        return true;
    }
    while (parser->cursor < parser->end && parse_string(parser)) {
        skip_whitespace(parser);
        if (parser->cursor >= parser->end || *parser->cursor++ != ':') {
            return false;
        }
        if (!parse_value(parser, depth + 1U)) {
            return false;
        }
        skip_whitespace(parser);
        if (parser->cursor >= parser->end) {
            return false;
        }
        char separator = *parser->cursor++;
        if (separator == '}') {
            return true;
        }
        if (separator != ',') {
            return false;
        }
        skip_whitespace(parser);
    }
    return false;
}

static bool parse_value(json_parser_t *parser, uint32_t depth)
{
    skip_whitespace(parser);
    if (parser->cursor >= parser->end) {
        return false;
    }
    switch (*parser->cursor) {
    case '{':
        return parse_object(parser, depth);
    case '[':
        return parse_array(parser, depth);
    case '"':
        return parse_string(parser);
    case 't':
        return consume_literal(parser, "true", 4U);
    case 'f':
        return consume_literal(parser, "false", 5U);
    case 'n':
        return consume_literal(parser, "null", 4U);
    default:
        return parse_number(parser);
    }
}

bool json_validate_object(const char *text, size_t length)
{
    if (!text || length == 0) {
        return false;
    }
    json_parser_t parser = {
        .cursor = text,
        .end = text + length,
    };
    skip_whitespace(&parser);
    if (parser.cursor >= parser.end || *parser.cursor != '{' ||
        !parse_object(&parser, 0)) {
        return false;
    }
    skip_whitespace(&parser);
    return parser.cursor == parser.end;
}
