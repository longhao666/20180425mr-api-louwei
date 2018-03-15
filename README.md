1.Download latest [peak can linux driver](https://www.peak-system.com/fileadmin/media/linux/index.htm#download "peak can linux driver") 
sudo apt-get install libpopt-dev
$ cd peak-linux-driver-8.x
$ make uninstall
2.Download latest PCAN-Basic API (Linux) https://www.peak-system.com/Support.55.0.html?&L=1
 extract it and install
3.cd mr-api
  mkdir build && cd build/
  cmake ..
  make && sudo make install
