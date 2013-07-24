Open Pixel Control Server
=========================

**WORK IN PROGRESS. NOT USABLE YET.**

The Fadecandy OPC Server is a daemon which glues one or more [Open Pixel Control](http://openpixelcontrol.org) clients to one or more supported hardware devices.

Clients include:

* Your own light effects code, written in any language
* The Fadecandy color balance tool

Supported hardware devices:

* Any number of Fadecandy boards, hot-pluggable via USB.
* [Enttec DMX USB Pro](http://www.enttec.com/?main_menu=Products&pn=70304) or compatible.

OPC Commands
------------

* Set pixel colors (0x00)
	* Standard OPC message. Data block is an array of 24-bit pixel colors.
* Set global color correction (0xF0)
	* Unofficial OPC message. Data block is JSON text, identical to the contents of the 'color' configuration key.


Configuration
-------------

The JSON configuration file is a dictionary which contains global configuration and an array of device objects. For each device, a dictionary includes device properties as well as a mapping table with commands which wire outputs to their corresponding OPC inputs. The map is a list of objects which act as mapping commands. Supported mapping objects:

* [ *OPC Channel*, *First OPC Pixel*, *First output pixel*, *pixel count* ]
	* Map a contiguous range of pixels from the specified OPC channel to the current device
	* For Fadecandy devices, output pixels are numbered from 0 through 511. Strand 1 begins at index 0, strand 2 begins at index 64, etc.

The following example config file supports two Fadecandy devices with distinct serial numbers. They both receive data from OPC channel #0. The first 512 pixels map to the first Fadecandy device. The next 64 pixels map to the entire first strand of the second Fadecandy device, and the next 32 pixels map to the beginning of the third strand.

	{
		"listen": ["127.0.0.1", 7890],
		"verbose": true,

		"color": {
			"gamma": 2.5,
			"whitepoint": [0.98, 1.0, 1.0]
		},

		"devices": [
			{
				"type": "fadecandy",
				"serial": "FFFFFFFFFFFF00180017200214134D44",
				"map": [
					[ 0, 0, 0, 512 ]
				]
			},
			{
				"type": "fadecandy",
				"serial": "FFFFFFFFFFFF0021003B200314134D44",
				"map": [
					[ 0, 512, 0, 64 ],
					[ 0, 576, 128, 32 ]
				]
			}
		]
	}

Prerequisites
-------------

The OPC server has been designed to work smoothly on Mac OS and Linux. With some work, it's possible to get it running on Windows.

* Requires [libusbx](https://github.com/libusbx/libusbx) version 1.0.16 or later.
* Requires [libev](http://software.schmorp.de/pkg/libev.html).

On Mac OS, the required packages are easy to install with [Homebrew](http://brew.sh/):

	$ brew install libusbx libev

On Debian or Ubuntu Linux, libev can be installed with apt-get, but currently you must compile libusbx yourself:

	$ sudo apt-get install libev libtool autoconf automake libudev-dev
	$ git clone https://github.com/libusbx/libusbx.git
	$ cd libusbx
	$ ./autogen.sh
	$ ./configure
	$ make
	$ sudo make install