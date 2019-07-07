#!/usr/bin/env bash

pushd $(dirname $BASH_SOURCE) > /dev/null

# C++ formatting
find src -iname "*.h" -o -iname "*.cpp" | xargs uncrustify --replace --no-backup -c uncrustify.cfg

# Python formatting
find src -iname "*.py" | xargs autopep8 --in-place --max-line-length 120

popd > /dev/null
