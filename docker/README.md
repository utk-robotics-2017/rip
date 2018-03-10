# RIP and Docker

How to use Docker to manage building and testing RIP.

## Installing Docker

You need to be using a Linux system in order to use Docker, on Ubuntu 

If you're on Ubuntu there are docker packages available in the multiverse, but the CE version from [the official website](https://docs.docker.com/install/linux/docker-ce/ubuntu/) will perform much better and offers newer features.

On MacOS, docker is available and will automatically run a Linux VM for you. On windows, you *can't* use the subsystem, since Docker depends on Linux kernel namespaces.

Once installed, be sure to do whatever group management to your account so that you can run docker without the need of sudo. (For CE, this is something like `adduser $USER docker` and then rebooting.)

## The Dependency Dockerfile

All of the useful RIP images are built from this one dependency image, which installs things like G2O and packages needed. You normally won't need to build this yourself.

To pull the latest deps image from dockerhub:

```docker pull utkrobotics/rip_deps```

This includes g2o, eigen, suitesparse, etc, whatever is needed to build RIP, so that you don't need to do the setup yourself.

## The interactive Dockerfiles

Since the RPI docker update, there are now two dockerfiles used for working with RIP builds interactively.

The easiest way to spin these up is to use the `./go-docker.sh` script which will handle everything for you.

### The Raspberry Pi emulating build

The option `rip_rpi` is a docker container that has a Raspi emulator installed, along with a virtual chroot and installed Pi dependencies.

Building this image normally takes a very long time, so it's recommended that you pull it from dockerhub instead, using the `update_rip_rpi` option in `go-docker.sh`.

Once you have the image locally, you can use the `go-docker.sh` script to start the container wherever you want. Whatever your current working directory is (`pwd`) the script will mount that into the container at `/workdir`. (Note: you shouldn't run the container outside of the `go-docker.sh` script since the container wouldn't be set up correctly.)

A shortcut to the rip_rpi container is to specify an entry command to the script:

```bash
# by default <command> is set to your $SHELL
./go-docker.sh --rpi -- <command> <args>
```

For example, if you just wanted to one-off the container to get a compiled Raspi build:

```bash
# `rpdo` runs the command on the ARM pi emulator inside the chroot (raspberry pi do)
./go-docker.sh --rpi -- rpdo ./build-linux.sh
```

And you don't have to stop at just RIP, after all, it's just a library. You can use the `go-docker.sh` script from anywhere else as well!

Example: compiling hugo and deploying it to a robot.

```bash
# cd to hugo's repo
# use the path to the dockerscript:
./rip/go-docker.sh --rpi
# (inside the container)
# `--no-cd` builds the current directory project instead of RIP
rpdo ./rip/build-linux.sh --no-cd
exit
# outside container
# we now have a build/ that has been built for a Raspberry pi!
scp -r build/ <pi> # or whatever you want to do to copy it over
```

### The 'blast-area' build (normal x86 / travis builds)

The normal rip image copies your current RIP source code into the image when it is built, and does not mount the working directory.

This is more useful when you have a radical idea you want to test, try out a failure or other non-ideal condition, or just want to otherwise test something without having it affect your current working directory of code.

In this container, there's no Raspberry Pi emulator, it only contains what is needed on x86_64 linux builds.

This is the build that travis currently uses to perform unit tests and coverage reports. The way it's used here is that the container is started once without the `--rm` flag so that multiple commands can be run on one instance of the container, using docker's exec tool.