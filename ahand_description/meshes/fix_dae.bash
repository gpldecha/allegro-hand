#!/bin/bash
find . -name \*.dae -exec sed -i "s/A_ONE/RGB_ZERO/g" {} \;
