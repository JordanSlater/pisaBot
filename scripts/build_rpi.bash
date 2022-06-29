#!/bin/bash
catkin_make --only-pkg-with-deps balance

## The above persists for other calls to catkin_make.
## To reset this build restriction, call:
# catkin_make -DCATKIN_WHITELIST_PACKAGES=""