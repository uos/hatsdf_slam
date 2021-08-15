#!/usr/bin/env bash

set -e

LIBUSB_DEBIAN="libusb-1.0-0-dev"
UDEV_DEBIAN="/etc/udev/rules.d/99-phidgets.rules"
LIBPHIDGET_VERSION="2.1.9.20190409"

announcement() {
  sleep 1
  echo ""
  echo "---- $1 ----"
}

if [[ -n "$(uname -a | grep Ubuntu)" ]]; then
  dpkg -l libusb-1.0-0-dev >/dev/null
  if [ ! $? -eq 0 ]; then
    announcement "Installing $LIBUSB_DEBIAN on Ubuntu host"
    sudo apt install "$LIBUSB_DEBIAN" -y
  fi

  if [[ ! -f "$UDEV_DEBIAN" ]]; then
    announcement "Setting up udev rules"
    sudo cp ./udev $UDEV_DEBIAN
    sudo udevadm control --reload-rules
    sudo udevadm trigger
  fi
  #pass # TODO implement on hardware platform
fi

if [ ! -f /usr/lib/libphidget21.so ]; then
  announcement "Installing phidgets c driver"
  if [ ! -f libphidget_"$LIBPHIDGET_VERSION".tar.gz ]; then
    wget https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget/libphidget_"$LIBPHIDGET_VERSION".tar.gz
    rm -rf libphidget_"$LIBPHIDGET_VERSION" >/dev/null
  fi

  tar -xvf libphidget_"$LIBPHIDGET_VERSION".tar.gz && cd libphidget-"$LIBPHIDGET_VERSION"
  ./configure && \
  make "CFLAGS=-g -O2 -Wno-incompatible-pointer-types -Wno-deprecated-declarations -Wno-format-truncation" && \
  sudo make install
fi

echo "done. Everything installed"