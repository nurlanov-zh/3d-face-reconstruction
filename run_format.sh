#!/usr/bin/env bash

set -e

find ./src -iname "*.hpp" -or -iname "*.h" -or -iname "*.cpp" -or -iname "*.cc" -or -iname "*.hh" | xargs clang-format -i
