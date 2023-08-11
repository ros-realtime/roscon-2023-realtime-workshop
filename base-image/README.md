Base image for the conference
=============================

To build, first install the [ros-realtime-rpi4-image][1] builder:

```
$ git clone https://github.com/ros-realtime/ros-realtime-rpi4-image
$ cd ros-realtime-rpi4-image
$ sudo python3 setup.py install
```

[1]: https://github.com/ros-realtime/ros-realtime-rpi4-image

Then, clone this repository, cd into the `base-image` directory and build the
image.

```
make image
```

Features
--------

- Raspberry Pi Ethernet port is always bound to `192.168.10.1` with the
  hostname `pi4.internal`.
  - `dnsmasq` acts as both the DHCP server and the DNS server. To make
    `dnsmasq` the DNS server, `systemd-resolved` had to be disabled.
- Any device that connects to the Raspberry Pi's Ethernet port will
  automatically get assigned an IP address via DHCP.
- The username and password is `ubuntu` and `hunter2`.
- If any USB Ethernet is connected to the Raspberry Pi, it will automatically
  configure via DHCP. This allows an escape hatch to connect to the Raspberry
  Pi by connecting it to a normal router via an USB Ethernet adapter.
  - The USB Ethernet interface is always named `usbeth`. Note: it is not a good
    idea to plug multiple USB Ethernet interface into Raspberry Pis with this
    image.

