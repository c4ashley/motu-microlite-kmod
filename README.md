# MOTU micro lite Linux Driver
Here is a loadable kernel module for the [MOTU micro lite][microlite] 5x5 USB MIDI Interface.
Written and tested on Fedora 41 with kernel version 6.11.8 for the x86_64 architecture.
Compatible with SecureBoot, so long as you are able to import your own signing keys into
your UEFI (scripts for generating keys and signing are included).

## Installing
> **WARNING:** Make sure you have a backup kernel available before installing modules
>permanently to your kernel. Installing initially failed for me, <u>and I was subsequently
>unable to boot into the latest kernel</u> (though I can't be certain the events are related).

### Without SecureBoot
Compilation and installation should work with no extra steps required.
```
make
make install
```

### With SecureBoot
You may find that you don't need to worry about any of this because signing is (I think)
a part of the installation process anyway. That said, even though I saw the signing steps
during installation, the module didn't seem to be want to load. Following are the steps I
used on my Fedora system. Different distributions may include different scripts in different
locations, and require different instructions. The routine in this repo makes use of
scripts provided by Fedora in `/usr/src/kernels/${uname -r}/scripts`. 
* If necessary, edit the `sign.sh` script according to your distribution's requirements.
* Edit the Makefile and uncomment the following line of the `all` recipe:
  * `./sign.sh microlite.ko # uncomment to sign on compilation`
* You will need to generate and import a signing key, if you do not have access to one:
  * `sudo make key`
  * Enter a password of your choice.
  * Reboot your computer.
  * Before your computer POSTs, you should be greeted with your UEFI to confirm importing
   the signing keys. Import the key and enter the password your entered earlier.
* Alternatively, if you _do_ have access to a signing key, you can modify the `sign.sh`
  file to use that instead of `motu.priv` and `motu.der`.
* Compilation and installation should now work as standard:
  ```
  make
  make install
  ```

## Loading
To load the driver immediately without installing it permanently, after running `make`
you can use `sudo make load` (or `modprobe microlite.ko`). You can also hot-unload or
hot-reload with `sudo make unload` and `sudo make reload` respectively, which unbinds the driver
from any connected micro lites, so you don't have to disconnect and reconnect the USB.

## Using
The driver will create an [ALSA rawmidi][rawmidi] hardware device with 5 subdevices
corresponding to each port. To find the IDs of the interfaces, use `amidi -l`. You can
also use them as [MIDI sequencer][sequencer] ports, for which you can find their IDs
with `aplaymidi -l`.

[microlite]: https://motu.com/products/midi/lite
[rawmidi]: https://www.alsa-project.org/alsa-doc/alsa-lib/rawmidi.html
[sequencer]: https://www.alsa-project.org/alsa-doc/alsa-lib/seq.html
