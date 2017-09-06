# RIP

The Robotics Integrated Platform is designed as an all encompassing library for small autonomous robots using a raspberry pi or beaglebone as the primary controller.

## How to Contribute

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
2. Build for what you're working on.  
   Replace `<TARGET>` with what you're building for. i.e. Arduino Gen is `ARDUINO_GEN`.
    * Linux: `cmake -D <TARGET>=yes ..`
    * Windows: `cmake -G "Unix Makefiles" -D <TARGET>=yes ..`

### Make
Type in `make` and you should start building rip, arduino_gen, etc.  
Make can be sped up by passing `-j<N>`, where is `N` is the number of cores on your computer.

## Core

## ArduionGen

## Appendages

## Utilities

## License
