#include <highfive/H5File.hpp>
#include <vector>
#include "catch2_config.h"

using namespace HighFive;

TEST_CASE("HighFive/HDF5 working", "[hdf5][HighFive]")
{
    std::cout << "Testing 'HighFive/HDF5 working'" << std::endl;
    // we create a new hdf5 file
    File file("/tmp/HighFiveTest.h5", File::ReadWrite | File::Create | File::Truncate);

    std::vector<int> data(50, 1);

    // let's create a dataset of native integer with the size of the vector 'data'
    DataSet dataset = file.createDataSet<int>("/dataset_one",  DataSpace::From(data));

    // let's write our vector of int to the HDF5 dataset
    dataset.write(data);

    // read back
    std::vector<int> result;
    dataset.read(result);
}