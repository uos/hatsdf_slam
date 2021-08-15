#pragma once

/**
 * @file pcd_file.h
 * @author Marc Eisoldt
 */

#include <string>
#include <fstream>
#include <map>
#include <memory>

#include <util/point.h>

namespace fastsense::util
{

class PCDFile
{
public:
    PCDFile(const std::string& file_name);

    void writePoints(const std::vector<std::vector<Vector3f>>& points, bool binary = false);

    void readPoints(std::vector<std::vector<Vector3f>>& points, unsigned int& number_of_points);

private:

    std::string name_;
};

} //namespace fastsense::util