DIR="$( cd "$( dirname "$0" )" && pwd )"

sudo apt-get install dpkg-dev
rm -fr ~/uvc_patch
mkdir -p ~/uvc_patch
cd ~/uvc_patch
apt-get source linux-image-$(uname -r)
sudo apt-get build-dep linux-image-$(uname -r)
sudo apt-get install linux-headers-$(uname -r)
cd linux-*
KBASE=`pwd`

patch -p1 < $DIR/../patches/linux-lts-saucy-3_11_0.patch
patch -p1 < $DIR/../patches/uvc_close_bulk_stream.patch

cp /boot/config-`uname -r` .config
cp  /usr/src/linux-headers-`uname -r`/Module.symvers .
make scripts oldconfig modules_prepare
cd drivers/media/usb/uvc
make -C $KBASE M=$KBASE/drivers/media/usb/uvc/ modules

## Copy module
sudo cp $KBASE/drivers/media/usb/uvc/uvcvideo.ko /lib/modules/`uname -r`/kernel/drivers/media/usb/uvc/uvcvideo.ko
## Remove old module if it's inserted
sudo rmmod uvcvideo.ko
