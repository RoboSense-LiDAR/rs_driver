#!bin/bash

# Stop on first error
set -o errexit
# Print each command ran
set -o xtrace

apt-get update

apt-get install -y --no-install-recommends \
   libboost-dev libboost-system-dev libpcap-dev
