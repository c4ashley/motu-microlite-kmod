obj-m += microlite.o
#CFLAGS_microlite.o := -v
#CFLAGS_microlite.o := -DDEBUG
KDIR := /lib/modules/$(shell uname -r)/build

all:
	#make -C $(KDIR) M=$(PWD) V=1 modules
	make -C $(KDIR) M=$(PWD) modules
	#./sign.sh microlite.ko # uncomment to sign on compilation
	chcon --reference=/usr/lib/modules microlite.ko

install: 
	make -C $(KDIR) M=$(PWD) V=1 modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean

load:
	modprobe ./microlite.ko

unload:
	./uninstall.sh; rmmod ./microlite.ko

key:
	echo "Make sure to run as sudo. If this step fails, read SIGNING.md and check with your distribution's guides on instructions for how to generate and import signing keys. This script was tested on Fedora, and other distros may have different scripts in different locations."
	openssl req -new -x509 -newkey rsa:2048 -keyout motu.priv -outform DER -out motu.der -nodes -days 36500 -subj "/CN=Mark of the Unicorn/"
	mokutil --import motu.der
	echo "Reboot and allow your UEFI to import the signing keys."

reload: unload load
