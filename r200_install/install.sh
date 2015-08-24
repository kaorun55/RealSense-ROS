echo "Installing DS4(R200) on your machine."
echo "----------------------------------------------------"
echo "1. Installing Driver"
echo "----------------------------------------------------"
sh ds_uvcdriver/scripts/patch_ubuntu_uvc.sh
echo "----------------------------------------------------"
echo "2. Copying Lib and include files"
echo "----------------------------------------------------"
sudo chmod -R 777 Lib
sudo chmod -R 777 Include

sudo mkdir /usr/include/dsapi
sudo cp -a Include/. /usr/include/dsapi
sudo cp -a Lib/. /usr/lib

echo "----------------------------------------------------"
echo "3. installing connectivity workaround rules"
echo "----------------------------------------------------"
cd ds_connectivity_workaround
sh install_rules.sh
cd ..
echo "----------------------------------------------------"
echo "Done. Please replug the DS4"
echo "----------------------------------------------------"
