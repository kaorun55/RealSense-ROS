echo "Installing DS4(R200) connectivity workaround. Please unplug any DS4 connected to you machine."
echo "Press Enter to continue..."
read p
echo "Copying R200 rule"
sudo cp 00-usb-R200.rules /etc/udev/rules.d/

echo "Copying scripts"
sudo cp usb-R200-in /usr/local/bin/
sudo cp usb-R200-in_udev /usr/local/bin/

echo "Making sure we have the right permissions"
sudo chmod 777 /usr/local/bin/usb-R200-in
sudo chmod 777 /usr/local/bin/usb-R200-in_udev
sudo chmod 777 /etc/udev/rules.d/00-usb-R200.rules

echo "Done. Please replug the DS4"
