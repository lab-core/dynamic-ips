#!/bin/bash

# Define variables
USER="amirelah"
SERVER="10.100.7.225"
REMOTE_PATH="/home/amirelah/Projects/dynamic-ips/DARP_IPS/datasets/Instances-120-in_Epoch"
LOCAL_PATH="/Users/ella/Projects"

# Step 1: Copy the directory
scp -r ${USER}@${SERVER}:${REMOTE_PATH} ${LOCAL_PATH}

# Step 2: Remove the directory from the server
ssh ${USER}@${SERVER} "rm -rf ${REMOTE_PATH}"