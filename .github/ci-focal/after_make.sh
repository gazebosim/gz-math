#!/bin/sh -l

set -x

# TODO(mjcarroll) Remove this script once codecheck is supported by focal CI
make codecheck
