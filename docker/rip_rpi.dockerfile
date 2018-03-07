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
# Place PI debootstrap images into external/rpi_images/*.tar.xz
ARG pi_image
ENV PI_IMAGE=${pi_image}
COPY external/rpi_images/${PI_IMAGE} /tmp/raspi-img.tar.xz
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
 && apt-get clean \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y apt-utils \
 && DEBIAN_FRONTEND=noninteractive dpkg --configure -a \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y '
RUN chroot $SYSROOT $QEMU_PATH /bin/sh -c '\
 DEBIAN_FRONTEND=noninteractive apt-get install -y \
  libc6-dev symlinks \
  libeigen3-dev libsuitesparse-dev qt5-default \
  bash zsh git vim tmux \
  cmake lcov g++ time libssh-dev unzip \
 && symlinks -cors / \
 && apt-get clean'

COPY docker/rpi/ /
RUN mkdir -p $SYSROOT/tmp
COPY external/g2o $SYSROOT/tmp/
RUN mkdir -p $SYSROOT/tmp/g2o/build
WORKDIR $SYSROOT/tmp/g2o/build
# ARM doesn't support SSE things and g2o's cmake can't detect that because /proc isn't mapped for ARM
RUN rpdo cmake .. -LA -DDISABLE_SSE2:BOOL=ON -DDISABLE_SSE3:BOOL=ON -DDISABLE_SSE4_1:BOOL=ON -DDISABLE_SSE4_2:BOOL=ON -DDISABLE_SSE4_A:BOOL=ON
RUN rpdo make -j$(nproc --ignore=1)
RUN rpdo make install
RUN rm -rf $SYSROOT/tmp/g2o

# requires a /dev mounted 'mount --bind /dev /rpxc/sysroot/dev'
# mounts require '--cap-add=SYS_ADMIN --security-opt apparmor:unconfined' args to docker run
#RUN install-raspbian python3

RUN mkdir -p $SYSROOT/workdir
WORKDIR /workdir
ENTRYPOINT [ "/rpxc/entrypoint.sh" ]

