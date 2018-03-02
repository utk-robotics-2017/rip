# RIP and Docker

How to use Docker to manage building and testing RIP.

Linux users also have access to the docker build script, build-docker.sh in the rip directory 

## Installing Docker

You can apt install docker.io, but this is out of date. The CE version is recommended.

If you're on Ubuntu there are docker packages available in the multiverse, but the CE version from [the official website](https://docs.docker.com/install/linux/docker-ce/ubuntu/) will perform much better and offers newer features.

On MacOS, docker is available and will automatically run a Linux VM for you. On windows, you *can't* use the subsystem, since Docker depends on Linux kernel namespaces.

Once installed, be sure to do whatever group management to your account so that you can run docker without the need of sudo. (For CE, this is something like `adduser $USER docker` and then rebooting.)

## The Dependency Dockerfile

Right now there are two dockerfiles, one that builds the dependencies (onto an Ubuntu base), and one that copies the source code of RIP.

The dependency image is sourced from `external/Dockerfile` and is built automatically by dockerhub, as `utkrobotics/rip_deps`.

To pull the latest deps image:

```docker pull utkrobotics/rip_deps```

This includes g2o, eigen, suitesparse, etc, whatever is needed to build.

## The building / interactive Dockerfile

This is the Dockerfile in the root of the repo.

It's based off the `rip_deps` image so it will have everything needed to build already installed.

Upon building this container image, it will copy all the source code from your current directory into the container and set up an environment ready to run the build.

To build the interactive container:

```
docker build --tag utkrobotics/rip:$(git symbolic-ref HEAD|cut -d'/' -f3-|sed -e 's;/;_;') .
```

This will create an image `utkrobotics/rip:yourbranch` which we can then run and create an instance of:

```
# --rm : removes the container when you exit it
# -t -i : creates an interactive container
# zsh -l : the command to run inside the container (use whatever your preferred shell is)
docker run --rm -t -i utkrobotics/rip:$(git symbolic-ref HEAD|cut -d'/' -f3-|sed -e 's;/;_;') zsh -l
```

Now that you're inside the container, you can immediately build it, mess around, do whatever, because everything you do inside the container is removed when you exit it, and it never affects the code outside the container.

Normally, what I do is:

```
# build dat stuff
./build-linux.sh
# runs the built unit tests
./travis/script.sh
```

While you're in the container, feel free to do whatever you want, `rm -rf *`, or anything. Just remember if you make changes that you want to keep inside the container, you should make those same changes again outside the container on your local git repo.

If you've made changed to your local git repo, you should run the build command again to update the image with your new changes.
