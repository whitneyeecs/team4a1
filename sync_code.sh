#!/bin/bash

rsync -pvrzt --progress --exclude "bin/*" --exclude "*.o" --exclude "*.a" * $1:eecs467/

