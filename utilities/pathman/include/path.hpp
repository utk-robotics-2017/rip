/**
 * PathMan
 * The Hero We Need
 *
 * @author UTK Robotics
 * @version 0.1
 */
#ifndef PATH_HPP
#define PATH_HPP

#include <string>
#include <vector>
#include <fstream>
#include <memory>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>


namespace pathman
{

    enum class PathType
    {
        NOT_SET,
        UNKNOWN,
        NOT_FOUND,
        FILE,
        DIRECTORY,
        ROOT,
        SYMLINK
    };

    class Path
    {
    public:
        /**
         * @brief Constructor
         */
        Path();

        /**
         * @brief Constructor
         * @param path String with relative or absolute path
         */
        Path(const std::string &const_path);

        /**
         * @brief Constructor
         * @param path Vector of strings representing a relative or absolute path
         * @param is_relative true if the path is relative, false otherwise
         */
        Path(std::vector<std::string> &path, bool is_relative = false);

        /**
         * @brief Copy Constructor
         * @param path
         */
        Path(const Path &path);

        /**
         * @brief Sets a new path for the Path object
         * @param path A string representing a new path
         */
        void setPath(const std::string &path);

        /**
         * @brief Determines if path is a directory
         * @return true if path is a directory, false otherwise
         */
        bool isDir() const;

        /**
         * @brief Determines if path is a file
         * @return true if path is a file, false otherwise
         */
        bool isFile() const;

        /**
         * @brief Determines if path exists
         * @return true if the path exists, false otherwise
         */
        bool exists() const;

        /**
         * @brief Get the file name of the current path
         * @return String of the file name or an empty string if the path type is not a file
         */
        std::string filename() const;

        /**
         * @brief Set the permissions of the current path
         * @param octal_code permissions code in octal (same as chmod)
         */
        void setPermissions(std::string &octal_code);

        /**
         * @brief Moves path up a directory
         */
        void upDir();

        /**
         * @brief Gets the parent directory of the current path
         * @return Path object for parent directory
         */
        Path getParent() const;

        /**
         * @brief Gets all of the files and directorys at the current path
         * @see pathman::Path::getChlidrenType()
         * @return A vector of path objects representing all the files/dirs at the current path
         */
        std::vector<Path> getChildren();

        /**
         * @brief Gets all of the sub directories at the current path
         * @see pathman::Path::getChlidrenType()
         * @return A vector of path objects representing all the dirs at the current path
         */
        std::vector<Path> getChildrenDirs();

        /**
         * @brief Gets all of the files at the current path
         * @see pathman::Path::getChlidrenType()
         * @return A vector of path objects representing all the files at the current path
         */
        std::vector<Path> getChildrenFiles();

        /**
         * @brief Gets all the symlinks at the current path
         * @return A vector of path objects representing all the symlinks at the current path
         */
        std::vector<Path> getChildrenSymlinks();

        /**
         * @brief Finds the specified filename recursively - DO NOT abuse
         * @param filename Name of the desired file
         * @param max_recursion Maximum levels of recursion to use when looking for the file
         * @return A path object with the file
         */
        std::unique_ptr<Path> find(const std::string &filename, int max_recursion);

        /**
         * @brief Creates a file at the current path if one does not exist
         */
        void createFile();

        /**
         * @brief Creates a directory at the current path if one does not exist
         */
        void createDir();

        /**
         * @brief Attempts to remove any file type at the current path
         * @param recursive recursively remove a directory (default false)
         */
        void removeAny(bool recursive = false);

        /**
         * @brief Remove the file at the current path
         */
        void removeFile();

        /**
         * @brief Removes the directory at the current path
         * @param force force the removal of the file/dir
         * @param recursive recursively remove the directory (default true)
         */
        void removeDir(bool recursive = true);

        /**
         * @brief Opens an input file stream for the current location
         * @return ifstream for the desired file
         */
        std::unique_ptr<std::ifstream> openInput();

        /**
         * @brief Opens an output file stream for the current location
         * @param create_if_does_not_exist Creates the file if it does not exist yet
         * @return ofstream for the desired file
         */
        std::unique_ptr<std::ofstream> openOutput(bool create_if_does_not_exist = false);

        /**
         * @brief Join a string to the end of the current path
         * @param path A relative path string to append to the current path
         */
        void join(const std::string &path);

        /**
         * @brief Overloaded += operator to append a path to the current path
         * @param rhs A string representing a relative path to append to the current path
         * @return The updated Path object
         */
        Path& operator+=(const std::string &rhs);

        /**
         * @brief Overloaded + operator to make a new path object when you add a string to a path
         * @param rhs A string to append to the current path
         * @return A new Path object with the updated path
         */
        Path operator+(const std::string &rhs);

        /**
         * @brief Resolve the file type for the path object
         * @return the type for the current path
         */
        PathType getType() const;

        /**
         * @brief Generates the string representation of the current path
         * @param relative Set to true to generate a relative path
         * @return A string of the current path
         */
        std::string str(bool relative = false) const;

        /**
         * @brief Generates a vector of strings representing the current path
         * @return A vector of strings for the current path
         */
        std::vector<std::string> vec() const;

    private:

        /**
         * current path stored as a string
         */
        std::string m_path_str;

        /**
         * current path stored as a vector
         */
        std::vector<std::string> m_path_components;

        /**
         * cache a copy of the stat struct
         */
        std::unique_ptr<struct stat> m_stat;

        /**
         * classification of the current path
         */
        PathType m_path_type;

        /**
         * @brief Gets all the children of a certain type in the current directory
         * @param types A vector of desired types (PathType enum)
         * @see pathman::PathType
         * @return A vector of paths matching the specified type in the current directory
         */
        std::vector<Path> getChildrenType(const std::vector<PathType> &types);

        /**
         * @brief Updates the internal stat structure
         */
        void updateStat();

        /**
         * @brief Invalidates any cached data in the Path object (e.g. changed path)
         */
        void invalidateCache();

        /**
         * @brief Updates the internal representation of the current path
         * @param path A string representing the new path
         */
        void updatePath(std::string path);

        /**
         * @brief Updates the internal representation of the current path
         * @param path A vector of strings representing the new path
         */
        void updatePath(std::vector<std::string> &path, bool is_relative = false);
    };
}

#endif //PATH_HPP
