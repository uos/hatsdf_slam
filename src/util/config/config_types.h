#pragma once

/**
 * @file config_types.h
 * @author Marcel Flottmann
 */

#include <unordered_map>
#include <boost/property_tree/ptree.hpp>

#include <util/event_handler_list.h>

namespace fastsense::util::config
{

class ConfigGroup;

/**
 * @brief Base class for configuration entries
 *
 */
class ConfigEntryBase
{
public:
    /**
     * @brief Default constructor
     *
     */
    ConfigEntryBase() = default;

    /**
     * @brief Default destructor
     *
     */
    virtual ~ConfigEntryBase() = default;

    /**
     * @brief Deleted copy constructor
     *
     */
    ConfigEntryBase(const ConfigEntryBase&) = delete;

    /**
     * @brief Deleted move constructor
     *
     */
    ConfigEntryBase(ConfigEntryBase&&) = delete;

    /**
     * @brief Deleted copy assignment
     *
     */
    ConfigEntryBase& operator=(const ConfigEntryBase&) = delete;

    /**
     * @brief Deleted move assignment
     *
     */
    ConfigEntryBase& operator=(ConfigEntryBase&&) = delete;

    /**
     * @brief Whether this entry is a group or not
     *
     * @return true *this is a group
     * @return false *this is not a group
     */
    virtual bool isGroup() = 0;

    /**
     * @brief Set current value or group members with data from property_tree
     *
     * @param val property_tree with the new data
     */
    virtual void set(const boost::property_tree::ptree& val) = 0;

    /**
     * @brief Check if current value or group members can be set with data from property_tree
     *
     * @param val property_tree with the new data
     * @return true Data is correct
     * @return false Data is not correct
     */
    virtual bool canSet(const boost::property_tree::ptree& val) = 0;

    /**
     * @brief Create property_tree node from this entry
     *
     * @return boost::property_tree::ptree node with data from this entry
     */
    virtual boost::property_tree::ptree getNode() = 0;
};

/**
 * @brief Concrete Entry with a value of type T
 *
 * @tparam T Type of this entry's value
 */
template<typename T>
class ConfigEntry : public ConfigEntryBase
{
    /// current value
    T value;
    /// mutex to lock when accessing value
    std::mutex mtx;
    /// Event handlers to call, when the value is updated
    EventHandlerList<void(T)> handlerList;
public:
    /**
     * @brief Construct a new Config Entry with name and parent
     *
     * @param name Name of the the entry
     * @param parent Parent of the entry or nullptr if this is top
     */
    ConfigEntry(std::string name, ConfigGroup* parent);

    /**
     * @brief Get current value
     *
     * @return T curretn value
     */
    T operator ()();

    /**
     * @brief Return that *this is not a group
     *
     * @return true never
     * @return false always
     */
    bool isGroup() override;

    /**
     * @brief Set current value with data from property_tree
     *
     * @param val property_tree with the new data
     */
    void set(const boost::property_tree::ptree& val) override;

    /**
     * @brief Check if current value can be set with data from property_tree
     *
     * @param val property_tree with the new data
     * @return true Data is correct
     * @return false Data is not correct
     */
    bool canSet(const boost::property_tree::ptree& val) override;

    /**
     * @brief Create property_tree node from this entry
     *
     * @return boost::property_tree::ptree node with data from this entry
     */
    boost::property_tree::ptree getNode() override;

    /**
     * @brief Add event handler when this entry is updated
     *
     * @param handler Callback to call when this entry is updated
     * @return EventHandlerHandle<void(T)> Handle to remove handler
     */
    EventHandlerHandle<void(T)> addHandler(const std::function<void(T)>& handler);

    /**
     * @brief Remove event handler
     *
     * @param handle Handle that was created when added
     */
    void removeHandler(EventHandlerHandle<void(T)>&& handle);
};

class ConfigGroup : public ConfigEntryBase
{
    std::unordered_map<std::string, ConfigEntryBase*> entries;
    EventHandlerList<void()> handlerList;

protected:
    template<typename T>
    friend class ConfigEntry;

    /**
     * @brief Add a member
     *
     * @param name
     * @param entry
     */
    void registerConfigEntry(std::string name, ConfigEntryBase* entry);

public:
    /**
     * @brief Construct a new Config Group with name and parent
     *
     * @param name Name of the the group
     * @param parent Parent of the group or nullptr if this is top
     */
    ConfigGroup(std::string name, ConfigGroup* parent);

    /**
     * @brief Return that *this is a group
     *
     * @return true always
     * @return false never
     */
    bool isGroup() override;

    /**
     * @brief Set group members with data from property_tree
     *
     * @param val property_tree with the new data
     */
    void set(const boost::property_tree::ptree& val) override;

    /**
     * @brief Check if group members can be set with data from property_tree
     *
     * @param val property_tree with the new data
     * @return true Data is correct
     * @return false Data is not correct
     */
    bool canSet(const boost::property_tree::ptree& val) override;

    /**
     * @brief Create property_tree node from group members
     *
     * @return boost::property_tree::ptree node with data from group members
     */
    boost::property_tree::ptree getNode() override;

    /**
     * @brief Add event handler when at least one group member is updated
     *
     * @param handler Callback to call when at least one group member is updated
     * @return EventHandlerHandle<void(T)> Handle to remove handler
     */
    EventHandlerHandle<void()> addHandler(const std::function<void()>& handler);

    /**
     * @brief Remove event handler
     *
     * @param handle Handle that was created when added
     */
    void removeHandler(EventHandlerHandle<void()>&& handle);
};

} // namespace fastsense::util::config

#define DECLARE_CONFIG_ENTRY(T, name, _documentation) ConfigEntry<T> name{#name, this}
#define DECLARE_CONFIG_GROUP(T, name) T name{#name, this}

#include "config_types.tcc"
