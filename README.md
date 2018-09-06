Wireless Bootloader
-------------------

![SMART Response XE](/bootloader_photo.jpg?raw=true "XE")

Copyright(c) 2018 BitBank Software, Inc.<br>
Written by Larry Bank<br>
bitbank@pobox.com<br>
<br>

This project consists of two parts: A custom bootloader for the SMART Response
XE handheld and an 'uploader' sketch which will run on another SMART Response XE
used as a 'hub' to connect to the PC.

The idea is to allow wireless uploading of Arduino sketches right from the IDE.
It was necessary to sacrifice an XE unit to act as the hub since there don't
appear to be inexpensive 802.15.4 (ZigBee) products which allow raw packet
manipulation.

The bootloader allows uploading of sketches through serial 0 or 1 and wirelessly. Since the SMART Response XE doesn't have a real power switch, the bootloader
code will put the unit into power down sleep after 2 minutes of no activity.
Press the wake button to wake it up.

I used the ATmega128RFA1 bootloader provided by Sparkfun as a starting point. I removed some unneeded parts and added my wireless code. The original copyright notice and comments are still there. My uploader sketch requires my XE support library which can be downloaded here:

https://github.com/bitbank2/SmartResponseXE

I've written a detailed blog post which documents the hows and whys of this
project here:

http://bitbanksoftware.blogspot.com/2018/09/my-adventures-in-writing-ota-bootloader.html






<br>
If you find this code useful, please consider buying me a cup of coffee<br>

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=SR4F44J2UR8S4)
