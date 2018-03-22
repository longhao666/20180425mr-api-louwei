### 1.Download latest [peak can linux driver](https://www.peak-system.com/fileadmin/media/linux/index.htm#download "peak can linux driver") 
`sudo apt-get install libpopt-dev`

`cd peak-linux-driver-8.x`

`make clean && make && sudo make install`

### 2.Download latest [PCAN-Basic API (Linux)](https://www.peak-system.com/Support.55.0.html?&L=1 "PCAN-Basic API (Linux)")
extract it and install

`cd PCAN_Basic_Linux-4.2.0/pcanbasic/`

`make clean && make && sudo make install`

`sudo modprobe pcan`

`ls /dev/`

### 3.Install mr-api
`cd mr-api/`

`mkdir build && cd build/`

`cmake ..`

`make clean && make && sudo make install`
