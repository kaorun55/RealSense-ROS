#!/bin/sh

red=$(tput setaf 1)
grn=$(tput setaf 2) 
NC=$(tput sgr 0) # No Color


yell() { echo "$0: $*" >&2; }

die() { yell "$*"; exit 111; }

try() { "$@" || die "$red Cannot $* $NC"; }

if [ -z $1 ]
then
	echo -e $(red)Provide an install path !!!$(NC)
	return 1
fi 

if [ ! -d $1 ]
then
	mkdir -p $1
fi

if [ -d $1 ]
then   	

	echo "$grn"
	echo "Welcome to ivcam basic IVCAM DLL installer $grn"
	echo "To continue you need your$red sudo$grn password$NC"

	GU=$(groups | cut -f1 -d ' ')
	echo SUBSYSTEMS==\"usb\", MODE=\"666\", GROUP=\"$GU\" | sudo tee /etc/udev/rules.d/48-usb.rules > /dev/null
	echo $grn 

	sudo chmod -R 777 lib
	sudo chmod -R 777 include
	
	echo Copying ivcam library include files
	mkdir -p $1/include
	cp -v ./include/* $1/include/.
	sudo mkdir /usr/include/ivcam
	sudo cp -r ./include/* /usr/include/ivcam
	echo Installing ivcamlib
	try sudo cp -v ./lib/libivcam.so /usr/lib/.

	echo "Installing USB DLL for IVCAM"
	#try sudo apt-get install libusb-dev
	echo "Copying ivcam sample app sources to $1/ivcamsample/"
	mkdir -p $1/ivcamsample/
	try cp -v ./ivcamsample/ivcam_sample.cc $1/ivcamsample/.
	try cp -v ./ivcamsample/build.sh $1/ivcamsample/.
	echo "Building ivcam_sample"
	cd $1/ivcamsample
	. ./build.sh
	cd $OLDPWD 
	
	echo Done!!
	echo $red 
	echo Please reboot your system now
	
else
	echo "$redCan not create or read install directory !!"
fi 
	echo $NC
