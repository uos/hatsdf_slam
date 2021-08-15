#pragma once

/**
 * @file config_types.tcc
 * @author Marcel Flottmann
 */

#include <util/logging/logger.h>

namespace fastsense::util::config
{

template<typename T>
ConfigEntry<T>::ConfigEntry(std::string name, ConfigGroup* parent) : value(T())
{
    if (parent)
    {
        parent->registerConfigEntry(name, this);
    }
}

template<typename T>
T ConfigEntry<T>::operator ()()
{
    std::lock_guard lock(mtx);
    T copy(value);
    return copy;
}

template<typename T>
bool ConfigEntry<T>::isGroup()
{
    return false;
}

template<typename T>
void ConfigEntry<T>::set(const boost::property_tree::ptree& val)
{
    {
        std::lock_guard lock(mtx);
        value = val.get_value<T>();
    }
    handlerList.invoke((*this)());
}

template<typename T>
bool ConfigEntry<T>::canSet(const boost::property_tree::ptree& val)
{
    return val.get_value_optional<T>() != boost::none;
}

template<typename T>
boost::property_tree::ptree ConfigEntry<T>::getNode()
{
    boost::property_tree::ptree node;
    node.put_value(value);
    return node;
}

template<typename T>
EventHandlerHandle<void(T)> ConfigEntry<T>::addHandler(const std::function<void(T)>& handler)
{
    return handlerList.add(handler);
}

template<typename T>
void ConfigEntry<T>::removeHandler(EventHandlerHandle<void(T)>&& handle)
{
    handlerList.remove(std::move(handle));
}

inline void ConfigGroup::registerConfigEntry(std::string name, ConfigEntryBase* entry)
{
    entries.emplace(name, entry);
}

inline ConfigGroup::ConfigGroup(std::string name, ConfigGroup* parent) : entries{}, handlerList{}
{
    if (parent)
    {
        parent->registerConfigEntry(name, this);
    }
}

inline bool ConfigGroup::isGroup()
{
    return true;
}

inline void ConfigGroup::set(const boost::property_tree::ptree& val)
{
    for (auto& item : val)
    {
        ConfigEntryBase* entry = entries.at(item.first);
        entry->set(item.second);
    }
    handlerList.invoke();
}

inline bool ConfigGroup::canSet(const boost::property_tree::ptree& val)
{
    for (auto& item : val)
    {
        auto entry_it = entries.find(item.first);
        if (entry_it == entries.end())
        {
            fastsense::util::logging::Logger::error("Config entry \"", item.first, "\" does not exist!");
            return false;
        }
        ConfigEntryBase* entry = entry_it->second;
        if (!entry->canSet(item.second))
        {
            if (entry->isGroup())
            {
                fastsense::util::logging::Logger::error("Cannot set config entry in \"", item.first, "\"!");
            }
            else
            {
                fastsense::util::logging::Logger::error("Cannot set config entry \"", item.first, "\" with value \"", item.second.data(), "\"!");
            }
            return false;
        }
    }
    return true;
}

inline boost::property_tree::ptree ConfigGroup::getNode()
{
    boost::property_tree::ptree node;

    for (auto& entry : entries)
    {
        node.put_child(entry.first, entry.second->getNode());
    }

    return node;
}

inline EventHandlerHandle<void()> ConfigGroup::addHandler(const std::function<void()>& handler)
{
    return handlerList.add(handler);
}

inline void ConfigGroup::removeHandler(EventHandlerHandle<void()>&& handle)
{
    handlerList.remove(std::move(handle));
}

} // namespace fastsense::util::config
