#!/bin/bash

apt update -qq
apt -qq install --no-install-recommends -y apt-utils
echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
