/**
 * @file main.cpp
 * @author Julian Gaal
 * @date 2020-08-09
 */


#include <stdlib.h>

#include "catch2_config.h"
#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>

namespace fs = fastsense;

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test krnl_vadd.cpp", "[buffers]")
{
    std::cout << "Testing 'Test krnl_vadd.cpp'" << std::endl;
    const char* xclbinFilename = "FastSense_test.xclbin";

    fs::hw::FPGAManager::load_xclbin(xclbinFilename);

    fs::CommandQueuePtr q = fs::hw::FPGAManager::create_command_queue();

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.

    fs::buffer::InputBuffer<int> buffer_a{q, DATA_SIZE};
    fs::buffer::InputBuffer<int> buffer_b{q, DATA_SIZE};
    fs::buffer::OutputBuffer<int> buffer_result{q, DATA_SIZE};
    fs::buffer::InputOutputBuffer<int> buffer_io{q, DATA_SIZE};

    //setting input data
    for (int i = 0 ; i < DATA_SIZE; i++)
    {
        buffer_a[i] = 10;
        buffer_b[i] = 20;
        buffer_result[i] = 10;
        buffer_io[i] = 20;
    }

    SECTION("Test operator*")
    {
        std::cout << "    Section 'Test operator*'" << std::endl;
        auto it = buffer_a.begin();

        REQUIRE_EACH(buffer_a, 10);

        it = buffer_a.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 20;
            REQUIRE(*it == 20);
            ++it;
        }

        it = buffer_a.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 10;
            REQUIRE(*it == 10);
            ++it;
        }

        it = buffer_result.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 20;
            REQUIRE(*it == 20);
            ++it;
        }

        it = buffer_result.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 10;
            REQUIRE(*it == 10);
            ++it;
        }

        it = buffer_io.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 10;
            REQUIRE(*it == 10);
            ++it;
        }

        it = buffer_io.begin();
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            *it = 20;
            REQUIRE(*it == 20);
            ++it;
        }
    }

    SECTION("Test operator* (const)")
    {
        std::cout << "    Section 'Test operator* (const)'" << std::endl;
        const auto& it = buffer_a.cbegin();
        REQUIRE(*it == 10);
    }

    SECTION("Test operator==")
    {
        std::cout << "    Section 'Test operator=='" << std::endl;

        {
            const auto& it = buffer_a.cbegin();
            const auto& comp_it = buffer_a.cbegin();
            for (int i = 0 ; i < DATA_SIZE; i++)
            {
                REQUIRE(it == comp_it);
            }
        }

        {
            const auto& it = buffer_b.cbegin();
            const auto& comp_it = buffer_b.cbegin();
            for (int i = 0 ; i < DATA_SIZE; i++)
            {
                REQUIRE(it == comp_it);
            }
        }

        {
            const auto& it = buffer_result.cbegin();
            const auto& comp_it = buffer_result.cbegin();
            for (int i = 0 ; i < DATA_SIZE; i++)
            {
                REQUIRE(it == comp_it);
            }
        }

        {
            const auto& it = buffer_io.cbegin();
            const auto& comp_it = buffer_io.cbegin();
            for (int i = 0 ; i < DATA_SIZE; i++)
            {
                REQUIRE(it == comp_it);
            }
        }
    }

    SECTION("Test range based loops (const)")
    {
        std::cout << "    Section 'Test range based loops (const)'" << std::endl;
        for (const auto& it : buffer_a)
        {
            REQUIRE(it == 10);
        }

        for (const auto& it : buffer_b)
        {
            REQUIRE(it == 20);
        }

        for (const auto& it : buffer_result)
        {
            REQUIRE(it == 10);
        }

        for (const auto& it : buffer_io)
        {
            REQUIRE(it == 20);
        }
    }

    SECTION("Test range based loops")
    {
        std::cout << "    Section 'Test range based loops'" << std::endl;
        for (auto& it : buffer_a)
        {
            REQUIRE(it == 10);
        }

        for (auto& it : buffer_b)
        {
            REQUIRE(it == 20);
        }

        for (auto& it : buffer_result)
        {
            REQUIRE(it == 10);
        }

        for (auto& it : buffer_io)
        {
            REQUIRE(it == 20);
        }
    }

    SECTION("Test operator[]")
    {
        std::cout << "    Section 'Test operator[]'" << std::endl;
        for (int i = 0 ; i < DATA_SIZE; i++)
        {
            REQUIRE(buffer_a[i] == 10);
            REQUIRE(buffer_b[i] == 20);
            REQUIRE(buffer_result[i] == 10);
            REQUIRE(buffer_io[i] == 20);
        }

#pragma GCC diagnostic warning "-Warray-bounds"
        REQUIRE_THROWS_AS(buffer_a[-1], std::out_of_range);
#pragma GCC diagnostic warning "-Warray-bounds"
        REQUIRE_THROWS_AS(buffer_a[DATA_SIZE + 1], std::out_of_range);
    }

    SECTION("Test move operator")
    {
        std::cout << "    Section 'Test move'" << std::endl;

        {
            auto test = std::move(buffer_a);
            REQUIRE(buffer_a.getVirtualAddress() == nullptr);
            REQUIRE(buffer_a.getQueue() == nullptr);
            REQUIRE(buffer_a.getBuffer().get() == nullptr);
            REQUIRE(test.getVirtualAddress() != nullptr);
            REQUIRE(test.getQueue() != nullptr);
            REQUIRE(test.getBuffer().get() != nullptr);
            REQUIRE_EACH(test, 10);
        }

        {
            auto test = std::move(buffer_b);
            REQUIRE(buffer_b.getVirtualAddress() == nullptr);
            REQUIRE(buffer_b.getQueue() == nullptr);
            REQUIRE(buffer_b.getBuffer().get() == nullptr);
            REQUIRE(test.getVirtualAddress() != nullptr);
            REQUIRE(test.getQueue() != nullptr);
            REQUIRE(test.getBuffer().get() != nullptr);
            REQUIRE_EACH(test, 20);
        }

        {
            auto test = std::move(buffer_result);
            REQUIRE(buffer_b.getVirtualAddress() == nullptr);
            REQUIRE(buffer_b.getQueue() == nullptr);
            REQUIRE(buffer_b.getBuffer().get() == nullptr);
            REQUIRE(test.getVirtualAddress() != nullptr);
            REQUIRE(test.getQueue() != nullptr);
            REQUIRE(test.getBuffer().get() != nullptr);
            REQUIRE_EACH(test, 10);
        }

        {
            auto test = std::move(buffer_io);
            REQUIRE(buffer_b.getVirtualAddress() == nullptr);
            REQUIRE(buffer_b.getQueue() == nullptr);
            REQUIRE(buffer_b.getBuffer().get() == nullptr);
            REQUIRE(test.getVirtualAddress() != nullptr);
            REQUIRE(test.getQueue() != nullptr);
            REQUIRE(test.getBuffer().get() != nullptr);
            REQUIRE_EACH(test, 20);
        }
    }

    // fs::kernels::VaddKernel krnl_vadd{q};

    // krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
    // krnl_vadd.waitComplete();

    // //Verify the result
    // for (int i = 0; i < DATA_SIZE; i++)
    // {
    //     int host_result = buffer_a[i] + buffer_b[i];
    //     REQUIRE(buffer_result[i] == host_result);
    // }
}
