#! /bin/bash

LIDAPY_FRAMEWORK=$(find $HOME -type d -name lidapy-framework 2>/dev/null)

# Assumes script is being run from the scripts directory
cd $LIDAPY_FRAMEWORK/test/lidapy

# Uses unittest autodiscovery to find and run all unittests under the test directory
python -m unittest discover --pattern '*.py'
