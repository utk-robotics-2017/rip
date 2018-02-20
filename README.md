# RIP

[![codecov](https://codecov.io/gh/utk-robotics-2017/rip/branch/master/graph/badge.svg?token=KqhG5MRr9F)](https://codecov.io/gh/utk-robotics-2017/rip)

*Version 0.1*

The Robotics Integrated Platform is designed as an all encompassing library for small autonomous robots using a raspberry pi or beaglebone as the primary controller.

## Dependencies
There are two ways to deal with the dependencies of RIP.

1. Use the provided Docker.
2. Install the packages natively on your local machine.

### Docker
See the [Docker Readme](Docker.md).

### Native Install
RIP requires several dependencies -- namely Eigen 3, Suitesparse, G2O, CMake, and GCC 4.9+. RIP GUIs require QT5.

## How to Contribute
All contributions should be made through our [Git workflow](https://github.com/utk-robotics-2017/rip/wiki/Git-Workflow) and following our [coding standards](https://github.com/utk-robotics-2017/rip/wiki/Coding-Standards).

Once your contributions are ready to be reviewed, please open a pull request to the dev branch. Ideally, mark your PR with the associated issues and the target milestone/version of RIP.

## Quick Start

### Get a terminal
On Linux, you have a terminal.  
On Windows, you'll want to get a terminal, something besides Cmd or PowerShell.  
There are several options, `Bash on Ubuntu on Windows`, `Mingw`, `Msys2`. You'll also need some additional terminal programs, i.e. cmake, g++, lcov, etc.

### This repo requires ssh.
1. [Generate an ssh key](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/)
2. [Add the ssh key to github](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)
3. [Test your ssh connection to github](https://help.github.com/articles/testing-your-ssh-connection/)

### Clone the repo using ssh.
1. Click `Clone or download`
2. Clone with ssh. Make sure the box says `Clone with SSH` in the top left corner of the box, if it doesn't, click `Use SSH` in the top right corner to switch.
3. Click on the clipboard icon to copy the address to your clipboard.
4. In your favorite terminal, type in `git clone --recursive `, then paste the address from github, and hit enter.

### Switch to the branch you'll be working on.
If you'll be working on Arduino Gen, you would switch to one of the `arduino_gen` branches. For instance, if you'll be working on Arduino Gen Unit Tests, then you'll switch to `arduino_gen/unit_tests`.

### Setup CMake
1. Make a directory for the output files (i.e. `build` or `bin`), then go into it.
2. Build the project.
    * Linux: `cmake ..`
    * Windows: `cmake -G "Unix Makefiles" ..`

### Make
Type in `make` and you should start building rip, arduino_gen, etc.  
Make can be sped up by passing `-j<N>`, where is `N` is the number of cores on your computer.

### Build Script
Optionally, rather than setting up the CMake build yourself, you may opt to use the included `build-linux.sh` script which will automatically build RIP in `build/` with testing enabled.

## License
**The RIP License (Revision 0.3)**

This software is available without warranty and without support. Use at your own risk. Literally. It might delete your filesystem or eat your cat. As long as you retain this notice, you can do whatever you want with this. If we meet some day, you owe me a beer.

Go Vols!
