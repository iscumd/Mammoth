#!/usr/bin/env bash

if [ colcon_cd mammoth_description ] ; then
    cp -rp models/* $HOME/.ignition/fuel/
else
    echo "Failed to find mammoth_desciption package, or _colcon_cd_root environment variable is not set properly."
fi