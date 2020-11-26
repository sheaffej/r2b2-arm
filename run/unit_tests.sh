#!/usr/bin/env bash
MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

$MYDIR/shell.sh pytest -v /ros/src/r2b2_arm/tests/unit