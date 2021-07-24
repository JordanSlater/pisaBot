#!/bin/bash

catkin_make --only-pkg-with-deps logger

catkin_make -DCATKIN_WHITELIST_PACKAGES=""