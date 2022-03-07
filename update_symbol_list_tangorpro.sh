#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
DEVICE_KERNEL_BUILD_CONFIG=private/devices/google/tangorpro/build.config.tangorpro \
private/gs-google/update_symbol_list.sh "$@"
