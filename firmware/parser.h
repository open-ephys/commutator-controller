#pragma once

#include "tinyjson/tiny-json.h"

int read_json_from_stdin(json_t const *root);
int parse_turns(json_t const *root, double *turns);
