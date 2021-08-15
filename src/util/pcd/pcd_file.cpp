/**
 * @file pcd_file.cpp
 * @author Marc Eisoldt
 */

#include "pcd_file.h"
#include <utility>
#include <sstream>
#include <typeinfo>

namespace fastsense::util
{

PCDFile::PCDFile(const std::string& file_name) : name_(file_name)
{

}

void PCDFile::writePoints(const std::vector<std::vector<Vector3f>>& points, bool binary)
{
    std::ofstream file(name_);

    if (!file)
    {
        throw std::ifstream::failure("Error while opening file for writing");
    }

    size_t width = 0;
    std::stringstream data;

    // Generate data

    float x, y, z;

    for (size_t ring_number = 0; ring_number < points.size(); ++ring_number)
    {
        for (const auto& point : points[ring_number])
        {
            if (binary)
            {
                x = static_cast<float>(point[0]);
                y = static_cast<float>(point[1]);
                z = static_cast<float>(point[2]);

                data.write((const char*)(&x), sizeof(x));
                data.write((const char*)(&y), sizeof(y));
                data.write((const char*)(&z), sizeof(z));
                data.write((const char*)(&ring_number), sizeof(ring_number));
            }
            else
            {
                data << point[0] << ' ' << point[1] << ' ' << point[2] << ' ' << ring_number << '\n';
            }

            ++width;
        }
    }

    // Write header
    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z ring\n";
    file << "SIZE 4 4 4 2\n";
    file << "TYPE F F F I\n";
    file << "COUNT 1 1 1 1  \n";
    file << "WIDTH " << width << '\n';
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << width << '\n';

    if (binary)
    {
        file << "DATA binary\n";
    }
    else
    {
        file << "DATA ascii\n";
    }

    // Write data
    file << data.str();

    // Make sure that all data are written
    file.flush();

    if (!file)
    {
        throw std::ifstream::failure("Error while writing into file");
    }

    file.close();
}

void PCDFile::readPoints(std::vector<std::vector<Vector3f>>& points, unsigned int& number_of_points)
{
    points.clear();
    std::ifstream file(name_);

    bool binary = false;

    if (!file)
    {
        throw std::ifstream::failure("Error while opening file for reading");
    }

    std::string str;

    for (auto header_line = 1u; header_line <= 10; ++header_line)
    {
        do
        {
            std::getline(file, str);

            if (str.size() == 0)
            {
                throw std::ifstream::failure("Error: PCD header has the wrong format at header line " + header_line);
            }

        }
        while (str[0] == '#');

        if (header_line == 6)
        {
            std::string delimiter = " ";

            size_t pos = 0;
            std::string token;
            while ((pos = str.find(delimiter)) != std::string::npos)
            {
                token = str.substr(0, pos);
                str.erase(0, pos + delimiter.length());
            }

            number_of_points = std::stoi(str);
        }
    }

    binary = (str.find("binary") != std::string::npos);

    float x, y, z;
    short ring;

    while (!file.eof())
    {
        if (binary)
        {
            if (!file.read((char*)&x, sizeof(x)))
            {
                break;
            }

            if (!file.read((char*)&y, sizeof(y)))
            {
                throw std::ifstream::failure("Error: y component could not be read");
            }

            if (!file.read((char*)&z, sizeof(z)))
            {
                throw std::ifstream::failure("Error: z component could not be read");
            }

            if (!file.read((char*)&ring, sizeof(ring)))
            {
                throw std::ifstream::failure("Error: ring component could not be read");
            }
        }
        else
        {
            if (!(file >> x))
            {
                break;
            }

            if (!(file >> y))
            {
                throw std::ifstream::failure("Error: y component could not be read");
            }

            if (!(file >> z))
            {
                throw std::ifstream::failure("Error: z component could not be read");
            }

            if (!(file >> ring))
            {
                throw std::ifstream::failure("Error: ring component could not be read");
            }
        }

        while (static_cast<size_t>(ring) >= points.size())
        {
            points.push_back(std::vector<Vector3f>());
        }

        points[ring].push_back({x, y, z});
    }

    file.close();
}

} // namespace fastsense::util