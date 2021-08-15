#pragma once

#include <iostream>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <util/logging/logger.h>

namespace fastsense
{

inline bool readStdIn()
{
    struct pollfd pfd = { STDIN_FILENO, POLLIN, 0 };

    std::string line;
    int ret = 0;
    
    // timeout of 0 ms to be in line with line.get
    ret = poll(&pfd, 1, 0);
    if(ret == 1) 
    {
        std::getline(std::cin, line);
        return true;
    }
    else if(ret == -1)
    {
        std::cout << "Error: " << strerror(errno) << std::endl;
    }

    return false;
}

} // namespace fastsense