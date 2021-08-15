#pragma once

/**
 * @file fpga_manager.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <fstream>

#include <hw/opencl.h>

namespace fastsense::hw
{

/**
 * @brief Manages FPGA Resources
 * 
 * Handles
 * * xlcbin file loading
 * * device access
 * * context access
 * * startup/shutdown
 */
class FPGAManager
{
public:
    /// default destructor
    ~FPGAManager() = default;

    /// deleted copy constructor
    FPGAManager(const FPGAManager&) = delete;

    /// deleted move constructor
    FPGAManager(FPGAManager&&) = delete;

    /// deleted copy assignment operator
    FPGAManager& operator=(const FPGAManager&) = delete;

    /// deleted move assignment operator
    FPGAManager& operator=(FPGAManager&&) = delete;

    /**
     * @brief Load xlcbin into FPGA
     * 
     * @param xclbin_filename filename
     */
    static void load_xclbin(const std::string& xclbin_filename);

    /**
     * @brief Get the device object
     * 
     * @return const cl::Device& FPGA device
     */
    static const cl::Device& get_device();

    /**
     * @brief Get the context object
     * 
     * @return const cl::Context& FPGA context
     */
    static const cl::Context& get_context();

    /**
     * @brief Get the program object
     * 
     * @return const cl::Program& FPGA program
     */
    static const cl::Program& get_program();

    /**
     * @brief Create a command queue object (singleton)
     * 
     * @return CommandQueuePtr shared_ptr command queue
     */
    static CommandQueuePtr create_command_queue();

    /// release resources
    static void release();

private:
    /// private default constructor
    FPGAManager();

    /**
     * @brief get singleton instance of FPGAManager
     * 
     * @return std::unique_ptr<FPGAManager>& 
     */
    static std::unique_ptr<FPGAManager>& inst();

    /// saves devices
    std::vector<cl::Device> devices_;

    /// FPGA context
    cl::Context context_;

    /// FPGA program
    cl::Program program_;

    /// Initalize devices
    void init_devices();

    /// Init device context
    void init_context();

    /**
     * @brief Load program into FPGA
     * 
     * @param xclbin_filename path to file
     */
    void load_program(const std::string& xclbin_filename);
};

} // namespace fastsense::hw