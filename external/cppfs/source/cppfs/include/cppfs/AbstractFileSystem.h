
#pragma once


#include <string>

namespace cppfs
{


class FileHandle;


/**
*  @brief
*    Interface for accessing file systems
*/
class AbstractFileSystem
{
public:
    /**
    *  @brief
    *    Constructor
    */
    AbstractFileSystem();

    /**
    *  @brief
    *    Destructor
    */
    virtual ~AbstractFileSystem();

    /**
    *  @brief
    *    Open file or directory in file system
    *
    *  @param[in] path
    *    Path to file or directory
    */
    virtual FileHandle open(const std::string & path) = 0;

    /**
    *  @brief
    *    Open file or directory in file system
    *
    *  @param[in] path
    *    Path to file or directory
    */
    virtual FileHandle open(std::string && path) = 0;
};


} // namespace cppfs
