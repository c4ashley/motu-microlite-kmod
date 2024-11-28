Create signing keys:
* `$ openssl req -new -x509 -newkey rsa:2048 -keyout {NAME}.priv -outform DER -out {NAME}.der -nodes -days 36500 -subj "/CN={Descriptive Name}/"`

Sign the module:
* `# /usr/src/kernels/$(uname -r)/scripts/sign-file sha256 ./{NAME}.priv ./{NAME}.der {MODULE}`

Register the key with SecureBoot (note that a reboot will be required, and the UEFI will ask you to confirm importing the key).
* `# mokutil --import {NAME}.der`

------

Enable SecureBoot
* `# mokutil --enable-validation`
* Enter an arbitrary password
* Reboot
* Allow the UEFI to change the SecureBoot configuration
* Enter the arbitrary password from before, or characters at certain positions as prompted by the UEFI

Disable SecureBoot
* `# mokutil --disable-validation`
* Same as above
