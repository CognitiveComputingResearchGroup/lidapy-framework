#! /bin/bash

source ~/.lidapy/setup.bash

LIDAPY_FRAMEWORK=$(find $HOME -type d -name lidapy-framework 2>/dev/null)

# Assumes script is being run from the scripts directory
cd $LIDAPY_FRAMEWORK/test/lidapy

pwd

echo "PYTHONPATH: " $PYTHONPATH

echo "Sys Paths from Python"
python ../../scripts/test_syspath.py

# Uses unittest autodiscovery to find and run all unittests under the test directory
python -m unittest discover --pattern '*.py' -v
