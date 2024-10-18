#include "parser.h"

#include <stdio.h>

// JSON parsing
#define STR_BUFFER_BYTES    256
#define MAX_COMMANDS        256

// Initialize buffers used by the JSON parser
char str[STR_BUFFER_BYTES];
json_t buff[MAX_COMMANDS];

// TODO: For some reason this does not return. It used to when it was part of main
static char *getline() 
{
    char u, *p;
    for(p = str, u = getchar(); u!='\r' && p-str < STR_BUFFER_BYTES; u = getchar())  
    {
        putchar(*p++ = u);
    }
    *p = 0;  
    return str;
}

int read_json_from_stdin(json_t const *root)
{
    char *rv = getline();

    if (rv == NULL) {
        puts("Invalid JSON command string.");
        return -1;
    }

    // NB: this call modifies str, so dont try to use it again!
    root = json_create(str, buff, sizeof buff / sizeof * buff);

    if (root == NULL) {
        puts("Invalid JSON formatting.");
        return -2;
    }

    return 0;
}

int parse_turns(json_t const *root, double *turns)
{
    json_t const *turn_prop = json_getProperty(root, "turn");
    if (turn_prop == NULL) {
        return -1;
    }

    if (json_getType(turn_prop) != JSON_REAL) {
        return -2;
    }

    *turns = json_getReal(turn_prop);

    return 0;
}

// int parse_channels(json_t const *root, bit_arr_t *bit_arr, size_t bit_arr_len)
// {
//     json_t const *ch_prop = json_getProperty(root, "channels");
//     if (ch_prop == NULL) {
//         return -1;
//     }

//     if (json_getType(ch_prop) != JSON_ARRAY) {
//         puts("The \"channels\" property must be an array.");
//         return -2;
//     }

//     // Clear all bits
//     for (int i = 0; i < bit_arr_len; i++)
//         bit_arr[i] = 0;

//     for (json_t const *i = json_getChild(ch_prop); i; i = json_getSibling(i)) {

//         // Check type
//         if (json_getType(i) != JSON_INTEGER) {
//             puts("Elements of \"channels\" array must be integers.");
//             return -3;
//         }

//         // Get the bit index
//         int b = json_getInteger(i);

//         if (b < 0 || b >= (bit_arr_len * 8 * sizeof(bit_arr_t))) {
//             puts("Invalid channel number.");
//             return -4;
//         }

//         set_bit(bit_arr, b);
//     }

//     return 0;
// }
