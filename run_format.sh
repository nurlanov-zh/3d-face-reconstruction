#!/usr/bin/env bash

set -e

find ./src -iname "*.hpp" -or -iname "*.h" -or -iname "*.cpp" | xargs clang-format -i
