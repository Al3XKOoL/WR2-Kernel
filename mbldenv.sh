#!/bin/bash
# ##########################################################
# ALPS(Android4.1 based) build environment profile setting
# ##########################################################
# Overwrite JAVA_HOME environment variable setting if already exists
JAVA_HOME=/opt/jdk1.6.0_26
export JAVA_HOME

# Overwrite ANDROID_JAVA_HOME environment variable setting if already exists
ANDROID_JAVA_HOME=/opt/jdk1.6.0_26
export ANDROID_JAVA_HOME

# Add MediaTek developed Python libraries path into PYTHONPATH
if [ -z "$PYTHONPATH" ]; then
  PYTHONPATH=$PWD/mediatek/build/tools
else
  PYTHONPATH=$PWD/mediatek/build/tools:$PYTHONPATH
fi
export PYTHONPATH
