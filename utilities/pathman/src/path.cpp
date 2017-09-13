/**
 * PathMan
 * The Hero We Need
 *
 * @author UTK Robotics
 * @version 0.1
 */
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <cstring>
#include <dirent.h>

#if (_WIN32)
#include <direct.h>
#endif

#include "../include/path.hpp"

namespace rip
{
    namespace utilities
    {
        namespace pathman
        {
            Path::Path()
            {
                // Get current working directory
                std::string path = getcwd(nullptr, 0);
                updatePath(path);
            }

            Path::Path(const std::string& const_path)
            {
                std::string path = const_path;
                // Update the internal path to match
                updatePath(path);
            }

            Path::Path(std::vector<std::string>& path, bool is_relative)
            {
                // Update the internal path to match
                updatePath(path, is_relative);
            }

            // Copy Constructor
            Path::Path(const Path& path)
            {
                // get the path string and path vector
                m_path_str = path.str();
                m_path_components = path.vec();

                // don't try to copy the stat data
                updateStat();
            }

            void Path::setPath(const std::string& path)
            {
                updatePath(path);
            }

            bool Path::isDir() const
            {
                return m_path_type == PathType::DIRECTORY || m_path_type == PathType::ROOT;
            }

            bool Path::isFile() const
            {
                return m_path_type == PathType::FILE;
            }

            bool Path::exists() const
            {
                return m_path_type != PathType::NOT_FOUND;
            }

            std::string Path::filename() const
            {
                return m_path_components[m_path_components.size() - 1];
            }

            void Path::setPermissions(std::string& octal_code)
            {
                int chmod_rv;
                mode_t mode;

                // convert the string to the appropriate mode
                mode = strtol(octal_code.c_str(), NULL, 8);

                // set the new permissions
                chmod_rv = chmod(this->str().c_str(), mode);

                if (!chmod_rv)
                {
                    // Do something with the error
                }
            }

            Path Path::getParent() const
            {
                // Get the current path as a vector while leaving off the last element
                std::vector<std::string> new_path(
                    &this->m_path_components[0],
                    &this->m_path_components[this->m_path_components.size() - 2]
                );

                return Path(new_path, false);
            }

            std::vector<Path> Path::getChildren()
            {
                std::vector<PathType> types {PathType::DIRECTORY, PathType::FILE, PathType::SYMLINK};
                return getChildrenType(types);
            }

            std::vector<Path> Path::getChildrenDirs()
            {
                std::vector<PathType> types {PathType::DIRECTORY, PathType::FILE};
                return getChildrenType(types);
            }

            std::vector<Path> Path::getChildrenFiles()
            {
                std::vector<PathType> types {PathType::FILE};
                return getChildrenType(types);
            }

            std::vector<Path> Path::getChildrenSymlinks()
            {
                std::vector<PathType> types {PathType::SYMLINK};
                return getChildrenType(types);
            }

            std::vector<Path> Path::getChildrenType(const std::vector<PathType>& types)
            {
                std::vector<Path> children;

                // *** ENTER C LAND *** //
                DIR* directory;
                struct dirent* entry;

                directory = opendir(this->str().c_str());

                if (directory == NULL)
                {
                    // handle error
                }

                while (entry = readdir(directory))
                {
                    int ssize = strlen(entry->d_name);

                    if (strncmp(entry->d_name, ".",  ssize) != 0 &&
                            strncmp(entry->d_name, "..", ssize) != 0   )
                    {
                        Path p(entry->d_name);

                        if (std::find(types.begin(), types.end(), p.getType()) != types.end())
                        {
                            children.push_back(p);
                        }
                    }
                }

                closedir(directory);
                // *** EXIT C LAND *** //

                return children;
            }

            std::unique_ptr<Path> Path::find(const std::string& filename, int max_recursion)
            {
                // base case
                if (max_recursion < 0) { return nullptr; }

                // get the children from the current directory
                std::vector<Path> paths = getChildren();

                // loop through each child from the current directory
                for (Path& path : paths)
                {
                    if (path.isFile() && path.filename() == filename)
                    {
                        // if the path is a file that matches the desired name, return a pointer to it
                        return std::unique_ptr<Path>(new Path(path));
                    }
                    else if (path.isDir())
                    {
                        // if the path is a directory, recursively search it
                        std::unique_ptr<Path> p = path.find(filename, max_recursion - 1);

                        // return up the chain if we find what we are looking for
                        if (p != nullptr) { return p; }
                    }
                }

                // if we make it through the whole search and can't find it, return a nullptr
                return nullptr;
            }

            void Path::createDir()
            {
                int mkdir_rv;
                // default permissions on a directory
                mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO;

                if (exists())
                {
                    // handle the error
                }

                #if defined(_WIN32)
                mkdir_rv = _mkdir(this->str().c_str());
                #else
                mkdir_rv = mkdir(this->str().c_str(), mode);
                #endif

                if (!mkdir_rv)
                {
                    // handle the error
                }

                updateStat();
            }

            void Path::createFile()
            {
                std::ofstream file;

                if (exists())
                {
                    // handle the error
                }

                // open a file at this path
                file.open(this->str());

                // create an empty file
                file << "";

                // close the file
                file.close();

                updateStat();
            }

            void Path::removeAny(bool recursive)
            {
                switch (m_path_type)
                {
                    case PathType::FILE :
                    case PathType::SYMLINK :
                    {
                        removeFile();
                        break;
                    }
                    case PathType::DIRECTORY :
                    {
                        removeDir(recursive);
                        break;
                    }
                    default:
                    {
                        // TODO - we don't try to delete other types (e.g. sockets)
                        break;
                    }
                }
            }

            void Path::removeFile()
            {
                if (!unlink(this->str().c_str()))
                {
                    // handle error
                }
                updateStat();
            }

            void Path::removeDir(bool recursive)
            {
                if (recursive)
                {
                    std::vector<Path> children = getChildren();
                    for (auto& path : children)
                    {
                        if (path.isDir())
                        {
                            path.removeDir(true);
                        }
                        else if (path.isFile())
                        {
                            path.removeFile();
                        }
                    }
                }

                if (!rmdir(this->str().c_str()))
                {
                    // handle error
                }
                updateStat();
            }

            std::unique_ptr<std::ifstream> Path::openInput()
            {
                if (!exists())
                {
                    // handle error
                }
                // TODO(Parker): I modifed this from this->str() to m_path_str so it would work
                // m_path_str is the absolute path and this->str() is returning a relative path
                return std::unique_ptr<std::ifstream>(new std::ifstream(m_path_str));
            }

            std::unique_ptr<std::ofstream> Path::openOutput(bool create_if_does_not_exist)
            {
                // if the file does not exist and should not be created
                if (!create_if_does_not_exist && !exists())
                {
                    // TODO
                }
                return std::unique_ptr<std::ofstream>(new std::ofstream(this->str()));
            }

            void Path::upDir()
            {
                // Can go up unless already at the root directory
                if (m_path_type != PathType::ROOT && m_path_type != PathType::UNKNOWN)
                {
                    // Invalidate the cached data
                    invalidateCache();

                    // Remove last element
                    m_path_components.pop_back();

                    // Update the path
                    updatePath(m_path_components);
                }
                else
                {
                    // Throw exception?
                    throw "Can't go up a directory";
                }
            }

            void Path::join(const std::string& path)
            {
                updatePath(this->str() + path);
            }

            Path& Path::operator+=(const std::string& rhs)
            {
                this->join(rhs);
                return *this;
            }

            Path Path::operator+(const std::string& rhs)
            {
                return Path(this->str() + rhs);
            }

            std::string Path::str(bool relative) const
            {
                std::stringstream ss;

                if (relative)
                {
                    // TODO
                }
                else
                {
                    for (auto& segment : m_path_components)
                    {
                        ss << "/" << segment;
                    }
                }

                return ss.str();
            }

            std::vector<std::string> Path::vec() const
            {
                return m_path_components;
            }

            PathType Path::getType() const
            {
                return m_path_type;
            }

            void Path::updateStat()
            {
                int stat_rv;

                // check if a stat struct should be allocated
                if (m_stat == nullptr)
                {
                    m_stat.reset(new struct stat);
                    //m_stat = std::make_unique<struct stat>(new struct stat);
                }

                // get the stat for the path
                // TODO(Parker): I changed this from this->str() to m_path_str
                // this->str() gives a relative path and m_path_str has the absolute
                stat_rv = stat(m_path_str.c_str(), m_stat.get());

                // if there was an error, determine why
                if ( stat_rv != 0 )
                {
                    switch (errno)
                    {
                        // Permissions issue
                        case EACCES:
                        {
                            // TODO: throw exception
                            throw "Permissions Error";
                            break;
                        }

                        // File does not exist
                        case ENOTDIR:
                        case ENOENT:
                        {
                            m_path_type = PathType::NOT_FOUND;
                            break;
                        }

                        // Weird things happened
                        case EBADF:
                        case EFAULT:
                        case ELOOP:
                        case ENOMEM:
                        case ENAMETOOLONG:
                        case EOVERFLOW:
                        default:
                        {
                            // TODO: throw an exception
                            throw "Bad things happened";
                            break;
                        }
                    }
                }

                // Determine the type of the path (file, dir, symlink, other)
                if(m_path_type != PathType::NOT_FOUND)
                {
                    switch (m_stat.get()->st_mode & S_IFMT) {
                        case S_IFDIR:
                        {
                            m_path_type = PathType::DIRECTORY;
                            break;
                        }

                        case S_IFREG:
                        {
                            m_path_type = PathType::FILE;
                            break;
                        }

                        #if defined(__linux)
                        case S_IFLNK:
                        {
                            m_path_type = PathType::SYMLINK;
                            break;
                        }

                        case S_IFBLK:  // block device
                        case S_IFSOCK: // socket
                        #endif
                        case S_IFCHR:  // char device
                        case S_IFIFO:  // fifo/pipe
                        default:
                        {
                            m_path_type = PathType::UNKNOWN;
                            break;
                        }
                    }
                }
            }

            void Path::updatePath(std::string path)
            {
                std::vector<std::string> path_vec;
                std::istringstream iss;
                std::string token;
                char path_cstr[PATH_MAX];

                // Clean the path string
                std::replace(path.begin(), path.end(), '\\', '/');
                #if defined(__linux__)
                realpath(path.c_str(), path_cstr);
                #else
                _fullpath(path_cstr, path.c_str(), PATH_MAX);
                #endif

                if (path_cstr == nullptr)
                {
                    // throw exception
                    throw "realpath failed";
                }

                // save the path string
                m_path_str = path_cstr;

                // clear out the old path vector
                m_path_components.clear();

                // Convert the string into a path vector
                iss.str(path);
                while (std::getline(iss, token, '/'))
                {
                    if (!token.empty())
                    {
                        m_path_components.push_back(std::move(token));
                    }
                }

                updateStat();
            }

            void Path::updatePath(std::vector<std::string> &path, bool is_relative)
            {
                std::stringstream ss;
                std::string path_str;
                std::string token;

                // safety check
                if(path.empty())
                {
                    // throw exception - because that is silly
                    throw "Empty Path variable";
                }

                // make the relative path absolute
                std::copy(path.begin(), path.end(), std::ostream_iterator<std::string>(ss, "/"));

                #if defined(__linux__)
                ss.str(realpath(ss.str().c_str(), nullptr));
                #elif defined(_WIN32)
                ss.str(_fullpath(nullptr, ss.str().c_str(), PATH_MAX));
                #endif

                // save the string
                m_path_str = ss.str();

                // Update the internal path vector
                while(std::getline(ss, token, '/'))
                {
                    if(!token.empty())
                    {
                        m_path_components.push_back(std::move(token));
                    }
                }

                updateStat();
            }

            void Path::invalidateCache()
            {
                m_stat.reset(nullptr);
                m_path_type = PathType::NOT_SET;
            }
        }
    }
}
