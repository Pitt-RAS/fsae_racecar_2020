#!/usr/bin/env bash

pushd $(dirname $0) > /dev/null

set -e

./compile.sh

teensy_loader_cli -mmcu=mk20dx256 -s -w /tmp/fsae-build/magellan_controller.ino.hex

popd > /dev/null
