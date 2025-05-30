#!/bin/bash

# creates the Purte Data extension based on the main branch on github
# script has to be executable

set -e

# remounting root filesystem read/write...
sudo mount -o remount,rw /

# removing old adsr directory
rm -r -f adsr

# cloning latest adsr from GitHub...
GIT_SSL_NO_VERIFY=true git clone https://github.com/attackallmonsters/adsr.git

# building release version
cd adsr || { echo "directory adsr not found"; exit 1; }
make release

echo "build complete"
