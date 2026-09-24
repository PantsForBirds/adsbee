#!/bin/bash

# Command line options documented here: https://kb.segger.com/J-Link_GDB_Server#Command_line_options

jlinkgdbserver -device CC1314R10 -endian little -speed 4000 -if cJTAG -port 2337