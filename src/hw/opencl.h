#pragma once

/**
 * @file opencl.h
 * @author Marcel Flottmann
 *
 * Include this file instead of <CL/cl2.hpp> directly to define version and other settings
 */

#define CL_HPP_CL_1_2_DEFAULT_BUILD
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY 1

#include <CL/cl2.hpp>
#include <memory>

namespace fastsense
{
using CommandQueuePtr = std::shared_ptr<cl::CommandQueue>;
}
