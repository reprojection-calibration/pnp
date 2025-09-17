#!/bin/bash

set -eoux pipefail

# Shellcheck
find /temporary/building -iname '*.sh' -print0 | xargs --null shellcheck
# Clang format
find /temporary/code \( -iname '*.cpp' -o -iname '*.hpp' \) -print0 | xargs --null clang-format --dry-run --Werror
# Cppcheck
cppcheck /temporary/code --enable=all --error-exitcode=1 -I /temporary/code/include -I /temporary/code/src --suppress=missingIncludeSystem