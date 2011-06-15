#/bin/bash

# IMPORTANT: This script assumes that you have downloaded and installed
# the khepera3toolbox and korebot2-oetools to $K3_ROOT and $K3_CROSS_ROOT,
# respectively.

export K3_CROSS_ROOT=/home/`whoami`/korebot2-oetools-1.0/tmp/cross
export K3_ROOT=/home/`whoami`/khepera3toolbox
export PATH=$PATH:$K3_ROOT/Scripts:$K3_CROSS_ROOT/bin

make -f hwdrv/Makefile

# IMPORANT: This script does NOT install the k3driver to the robot
# use 'k3put +ID hwdrv/k3driver' to install it to a K3.
