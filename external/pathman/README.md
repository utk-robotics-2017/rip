# pathman
Path Manager for RIP (C++11)

"It isn't the hero we want, but it is the hero we deserve."

The library primarily is contained in the Path class. The Path class implements all the necessary methods for basic directory and file actions on POSIX systems in a C++ manner to abstract away the C POSIX API details. 

_Note: We use C++ exceptions rather than return values for dealing with failure cases._

## How to Use
Start by instantiating a Path object.
```
// Creates a path object for my home directory
Path home_path("/home/parker");

// Creates a path object for the current directory of the executable
Path current_path();

// Creates a path object for a file relative to the current directory
Path my_file("a_file.txt");
```

With this object, there are a number of actions you may take. Below, there are a few listed examples.

### General Operation

* Get current path as a string `my_path.str();`
* Check if path exists `my_path.exists();`
* Change the permissions `my_path.changePermissions("0755");`
* Change the path `my_path.setPath("/opt/cool_software");`

### Directory Operations

* Check if path is a directory `my_path.isDir();`
* Get a vector of paths within a directory `my_path.getChildren();`
* Create a directory at the current path `my_path.createDir();`
* Remove a directory (with -r) `my_path.removeDir(true);`
* Get parent directory as Path object `my_path.getParent();`

### File Operations

* Check if path is a file `my_path.isFile();`
* Create a file `my_path.createFile();`
* Delete current file `my_path.removeFile();`
* Open an output file stream `my_path.openOutput();`
* Open an input file stream `my_path.openInput();`

## Application Example
_TODO_