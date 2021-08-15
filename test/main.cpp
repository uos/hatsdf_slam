/**
 * @author Marcel Flottmann
 */

#define CATCH_CONFIG_RUNNER
#include "catch2_config.h"

#include <hw/fpga_manager.h>
#include <util/logging/logger.h>

using namespace fastsense::util::logging;
using namespace fastsense::hw;

int main( int argc, char* argv[] )
{
    auto logSink = std::make_shared<sink::FileSink>("test.log");
    Logger::addSink(logSink);
    Logger::setLoglevel(LogLevel::Debug);

    FPGAManager::load_xclbin("FastSense_test.xclbin");

    int result = Catch::Session().run(argc, argv);

    FPGAManager::release();

    return result;
}
