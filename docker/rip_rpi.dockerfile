FROM utkrobotics/rip_deps:latest
# FROM RUSER 902:902

USER root

# Here is where we hardcode the toolchain decision.
ENV HOST=arm-linux-gnueabihf \
    TOOLCHAIN=gcc-linaro-arm-linux-gnueabihf-raspbian-x64 \
    RPXC_ROOT=/rpxc
#    TOOLCHAIN=arm-rpi-4.9.3-linux-gnueabihf \
#    TOOLCHAIN=gcc-linaro-arm-linux-gnueabihf-raspbian-x64 \

WORKDIR $RPXC_ROOT
RUN curl -L https://github.com/raspberrypi/tools/tarball/master \
  | tar --wildcards --strip-components 3 -xzf - "*/arm-bcm2708/$TOOLCHAIN/"

ENV ARCH=arm \
    CROSS_COMPILE=$RPXC_ROOT/bin/$HOST- \
    PATH=$RPXC_ROOT/bin:$PATH \
    QEMU_PATH=/usr/bin/qemu-arm-static \
    QEMU_EXECVE=1 \
    SYSROOT=$RPXC_ROOT/sysroot

WORKDIR $SYSROOT
ARG pi_image
ENV PI_IMAGE=${pi_image}
COPY rpi_images/${PI_IMAGE} /tmp/raspi-img.tar.xz
# extract the raspi debootstrap image into our chroot
RUN cat /tmp/raspi-img.tar.xz \
    | tar -xJf -
# get the QEMU ARM emulator
RUN curl -fsSL https://github.com/resin-io-projects/armv7hf-debian-qemu/raw/master/bin/qemu-arm-static \
    > $SYSROOT/$QEMU_PATH
RUN chmod +x $SYSROOT/$QEMU_PATH
RUN chroot $SYSROOT $QEMU_PATH /bin/sh -c '\
 echo "deb http://mirrordirector.raspbian.org/raspbian/ jessie main contrib non-free rpi" > /etc/apt/sources.list \
 && echo "deb http://archive.raspberrypi.org/debian/ jessie main ui" >> /etc/apt/sources.list \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 82B129927FA3303E \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y apt-utils \
 && DEBIAN_FRONTEND=noninteractive dpkg-reconfigure apt-utils \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y '
RUN chroot $SYSROOT $QEMU_PATH /bin/sh -c '\
 DEBIAN_FRONTEND=noninteractive apt-get install -y \
  libc6-dev symlinks python3 \
  libeigen3-dev libsuitesparse-dev \
  bash zsh git vim tmux \
  cmake lcov g++ time libssh-dev unzip \
 && symlinks -cors / \
 && apt-get clean'

COPY rpi/ /

RUN mkdir -p $SYSROOT/workdir
WORKDIR /workdir
ENTRYPOINT [ "/rpxc/entrypoint.sh" ]
