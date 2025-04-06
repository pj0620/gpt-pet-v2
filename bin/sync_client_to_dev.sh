#!/bin/bash

rsync --rsync-path="sudo rsync" -avz --exclude=/dev --exclude=/proc --exclude=/sys --exclude=/tmp --exclude=/run --exclude=/mnt --exclude=/media --exclude=/lost+found gptpetclient2@gptpetclient2.local:/ ~/sysroot/