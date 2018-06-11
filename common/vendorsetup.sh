# *********************************************************************
# Copyright (c) 2013 Qualcomm Technologies, Inc.  All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.
# *********************************************************************

# QC specific definitions & variables

export QCPATH="vendor/qcom/proprietary"

if [ $(uname -s) = Linux ]; then
XCOMP64=`pwd`/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.8/bin
fi

if [ -d "$XCOMP64" ]; then
  XCOMP64_INPATH=$(echo "$PATH" | grep "$XCOMP64" | head -n1)
  if [ -z "$XCOMP64_INPATH" ]; then
    echo "Adding 64-bit toolchain to path ($XCOMP64)"
    export PATH="$PATH:$XCOMP64"
  fi
fi
