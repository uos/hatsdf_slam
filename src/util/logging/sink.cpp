/**
 * @file sink.cpp
 * @author Marcel Flottmann
 */

#include <iostream>

#include "sink.h"

using namespace fastsense::util::logging::sink;

void CoutSink::write(const std::string& msg)
{
    std::cout << msg;
}

FileSink::FileSink(const std::string& filename, std::ios_base::openmode mode) : file(filename, mode)
{
}

void FileSink::write(const std::string& msg)
{
    file << msg;
    file.flush();
}
