#pragma once

/**
 * @file config_manager.h
 * @author Marcel Flottmann
 */

#include "config.h"

namespace fastsense::util::config
{

/**
 * @brief Implementation of the Configuration Manager
 * for loading updating and storing the configuration
 *
 * @tparam T Type of the configuration data. Must be of derived from ConfigEntryBase
 */
template<typename T>
class ConfigManagerImpl
{
private:
    /// The configuration data
    T configData;

    /**
     * @brief update the configuration with data from boost porperty_tree
     *
     * @param tree property_tree that holds the new data
     */
    static void update(const boost::property_tree::ptree& tree);

    /**
     * @brief Private constructor to create singleton instance
     *
     */
    ConfigManagerImpl();

    /**
     * @brief Get the singleton instance
     *
     * @return ConfigManagerImpl<T>& reference to the singleton
     */
    static ConfigManagerImpl<T>& getInst();
public:

    /**
     * @brief Load configuration data from JSON file
     * The file content can be a partial (not all configuration entries may be updated)
     *
     * @param filename JSON file to read
     */
    static void loadFile(const std::string& filename);

    /**
     * @brief Load configuration data from JSON string
     * The file content can be a partial (not all configuration entries may be updated)
     *
     * @param data string that contains data in JSON format
     */
    static void loadString(const std::string& data);

    /**
     * @brief Create a JSON string from current configuration
     *
     * @return std::string Current configuration in JSON format
     */
    static std::string createString();

    /**
     * @brief Create JSON file from current configuration
     *
     * @param filename File to write
     */
    static void writeFile(const std::string& filename);

    /**
     * @brief Get the current configuration
     *
     * @return T& Reference to the current configuration
     */
    static T& config();
};

/**
 * @brief Specialisation with global configuration of the ConfigManager
 *
 */
using ConfigManager = ConfigManagerImpl<Config>;

} // namespace fastsense::util::config

#include "config_manager.tcc"
