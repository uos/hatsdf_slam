/**
 * @file hw_buffers.cpp
 * @author Julian Gaal
 */

#include <iostream>
#include "kernels/io_buffer_test_kernel.h"
#include "kernels/vadd_kernel.h"
#include "catch2_config.h"

using namespace fastsense;

TEST_CASE("Testing HW Buffers", "[TestHWBuffers]")
{
    auto q = hw::FPGAManager::create_command_queue();
    kernels::IOBufferTestKernel kernel{q};

    constexpr int DATA_SIZE = 100;
    buffer::InputOutputBuffer<int> data{q, DATA_SIZE};
    std::fill(data.begin(), data.end(), 1);

    kernel.run(data, DATA_SIZE);
    kernel.waitComplete();

    SECTION("Test 'InputOutputBuffer'")
    {
        std::cout << "    Section 'InputOutputBuffer'\n";
        REQUIRE_EACH(data, 2);
    }

    SECTION("Test 'InputBuffer and OutputBuffer separately'")
    {
        std::cout << "    Section 'InputBuffer and OutputBuffer separately'\n";

        // These commands will allocate memory on the Device. The cl::Buffer objects can
        // be used to reference the memory locations on the device.
        buffer::InputBuffer<int> buffer_a{q, DATA_SIZE};
        buffer::InputBuffer<int> buffer_b{q, DATA_SIZE};
        buffer::OutputBuffer<int> buffer_result{q, DATA_SIZE};

        //setting input data
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            buffer_a[i] = 10;
            buffer_b[i] = 20;
        }

        kernels::VaddKernel krnl_vadd{q};

        krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
        krnl_vadd.waitComplete();

        //Verify the result
        for (int i = 0; i < DATA_SIZE; i++)
        {
            int host_result = buffer_a[i] + buffer_b[i];
            REQUIRE(buffer_result[i] == host_result);
        }
    }
}