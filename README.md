# hsdaoh - High Speed Data Acquisition over HDMI

This project aims to (ab)use cheap USB 3.0 HDMI capture sticks based on the MacroSilicon MS2130 as a general purpose USB interface.

In combination with small FPGA boards with HDMI interface like the Tang Nano series, it can be used to capture high speed data streams from an external data source like an ADC, do-it-yourself SDR, or simply build a high speed logic analyzer.

For more information, please take a look at the slides of the presentation at OsmoDevcon 2024 [here](https://people.osmocom.org/steve-m/hsdaoh_slides/rendered/osmodevcon2024_hsdaoh.pdf).
A proper video recording will be available soon, for now there is only a [streamdump](https://relive.c3voc.de/relive/osmodevcon24/200/muxed.mp4) (the actual talk starts at 17:23).

## Getting started

You need at least two pieces of hardware:

### HDMI Capture device based on MS2130 or MS2131
There are many sources for those devices, ranging from Aliexpress to Amazon or maybe even your local hardware retailer, the price is typically around USD 10. Simply search for MS2130.
Just make sure that it is marked as a real USB 3.0 device (most of them are marked as "U3" on the case), as cheaper sticks use the older MS2109 chip which is USB 2.0 only.

### FPGA board with HDMI out
The main target are currently the Tang Nano series of boards (4K, 9K and 20K). The Tang Primer boards as well as the EBAZ4205 also have been tested.

## First run

- Please follow the instructions at https://github.com/steve-m/hsdaoh-fpga to get an example design running on your FPGA.
- Follow the instructions below to get hsdaoh running on your computer.
- Then connect your FPGA board to the HDMI grabber and make sure you can see actual video output.
- You then can use hsdaoh_test to verify the counter values being output by the FPGA.

## Installation

### Linux

As this project shares code with [rtl-sdr](https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr) and [osmo-fl2k](https://osmocom.org/projects/osmo-fl2k/wiki), the installation process is almost identical.
There is one additional dependency to abstract the video input however (libuvc).

To install the build dependencies on a distribution based on Debian (e.g. Ubuntu), run the following command:

    sudo apt-get install build-essential cmake pkgconf libusb-1.0-0-dev libuvc-dev

To build hsdaoh:

    git clone https://github.com/steve-m/hsdaoh.git
    mkdir hsdaoh/build
    cd hsdaoh/build
    cmake ../ -DINSTALL_UDEV_RULES=ON
    make -j 4
    sudo make install
    sudo ldconfig

To be able to access the USB device as non-root, the udev rules need to be installed (either use -DINSTALL_UDEV_RULES=ON or manually copy hsdaoh.rules to /etc/udev/rules.d/).

Before being able to use the device as a non-root user, the udev rules need to be reloaded:

    sudo udevadm control -R
    sudo udevadm trigger

Furthermore, make sure your user is a member of the group 'plugdev'.
To make sure the group exists and add your user to it, run:

    sudo groupadd plugdev
    sudo usermod -a -G plugdev <your username>

If you haven't already been a member, you need to logout and login again for the group membership to become effective.

### Windows and Mac OS X
As libuvc and libusb are cross-platform libraries, it should be able to build libhsdaoh for Win/OSX as well. As of now, this is untested.

## Applications

### hsdaoh_file

This application records the data to a file or FIFO.

### hsdaoh_tcp

This application is similar to rtl_tcp, it opens a listening TCP socket (by default on port 1234) and streams out the data if a client connects.

### hsdaoh_test

The purpose of this application is measuring the real rate the device outputs and verifying the test counter. It can be used to test if the device works correctly and if the clock is stable, and if there are any bottlenecks with the USB or HDMI connection.

## Credits

hsdaoh is developed by Steve Markgraf, and is heavily based on rtl-sdr and osmo-fl2k.
