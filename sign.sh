# Sign the kernel module for use with SecureBoot enabled operating systems.
# Tested on Fedora. Other distros may have different scripts in a different location.
/usr/src/kernels/$(uname -r)/scripts/sign-file sha256 ./motu.priv ./motu.der $1
