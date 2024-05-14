#!/bin/sh

export DRIVEWORKS_WORKSPACE=/srv/nfs/driveworks-aarch64-linux-gnu
export IP_ADDR=172.18.10.67
export HOSTNAME=$(hostname)

orin_exec () {
  ssh -tt wayve@$IP_ADDR $*
}

# Exit on error
set -e

# Install SSH key (to avoid need to enter password for every command).
ssh-copy-id wayve@$IP_ADDR 

orin_exec "sudo mkdir -p /srv/nfs/driveworks-aarch64-linux-gnu && sudo mount -t nfs $HOSTNAME:/srv/nfs/driveworks-aarch64-linux-gnu /srv/nfs/driveworks-aarch64-linux-gnu"
orin_exec "sudo mkdir -p /usr/local/driveworks/samples/bin && sudo mount $HOSTNAME:$DRIVEWORKS_WORKSPACE/build-aarch64-linux-gnu/install/usr/local/driveworks/samples/bin /usr/local/driveworks/samples/bin"
orin_exec "sudo mkdir -p /usr/local/driveworks/data && sudo mount $HOSTNAME:/usr/local/driveworks/data /usr/local/driveworks/data"
