#pragma once

/**
 * @file config_manager.tcc
 * @author Marcel Flottmann
 */

#include <boost/property_tree/json_parser.hpp>
#include <type_traits>

namespace fastsense::util::config
{

template<typename T>
ConfigManagerImpl<T>::ConfigManagerImpl() : configData(std::string(), nullptr)
{
    static_assert(std::is_base_of_v<ConfigEntryBase, T>);
}

template<typename T>
ConfigManagerImpl<T>& ConfigManagerImpl<T>::getInst()
{
    static ConfigManagerImpl<T> inst;
    return inst;
}

template<typename T>
void ConfigManagerImpl<T>::loadFile(const std::string& filename)
{
    boost::property_tree::ptree tree;
    boost::property_tree::read_json(filename, tree);
    update(tree);
}

template<typename T>
void ConfigManagerImpl<T>::loadString(const std::string& data)
{
    boost::property_tree::ptree tree;
    std::istringstream issdata(data);
    boost::property_tree::read_json(issdata, tree);
    update(tree);
}


template<typename T>
std::string ConfigManagerImpl<T>::createString()
{
    auto tree = config().getNode();
    std::ostringstream oss;
    boost::property_tree::write_json(oss, tree);
    return oss.str();
}

template<typename T>
void ConfigManagerImpl<T>::writeFile(const std::string& filename)
{
    auto tree = config().getNode();
    boost::property_tree::write_json(filename, tree);
}

template<typename T>
void ConfigManagerImpl<T>::update(const boost::property_tree::ptree& tree)
{
    if (config().canSet(tree))
    {
        config().set(tree);
    }
    else
    {
        throw std::runtime_error("Could not set configuration");
    }
}

template<typename T>
T& ConfigManagerImpl<T>::config()
{
    return getInst().configData;
}

} //namespace fastsense::util::config