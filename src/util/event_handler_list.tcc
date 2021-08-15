#pragma once

/**
 * @file event_handler_list.tcc
 * @author Marcel Flottmann
 */

namespace fastsense::util
{

template<typename T>
EventHandlerHandle<T>::EventHandlerHandle(typename std::list<std::function<T>>::iterator it) : iterator(it)
{}

template<typename T>
EventHandlerHandle<T>::EventHandlerHandle(EventHandlerHandle&& rhs) noexcept :  iterator(rhs.iterator)
{
    rhs.iterator = std::list<std::function<T>>::iterator();
}

template<typename T>
EventHandlerHandle<T>& EventHandlerHandle<T>::operator=(EventHandlerHandle&& rhs) noexcept
{
    iterator = rhs.iterator;
    rhs.iterator = std::list<std::function<T>>::iterator();
    return *this;
}

template<typename T>
EventHandlerHandle<T> EventHandlerList<T>::add(const std::function<T>& callback)
{
    std::lock_guard lock(mtx);
    auto it = callbacks.insert(callbacks.end(), callback);
    return EventHandlerHandle<T>(it);
}

template<typename T>
void EventHandlerList<T>::remove(EventHandlerHandle<T>&& handle)
{
    if (handle.iterator != typename std::list<std::function<T>>::iterator())
    {
        std::lock_guard lock(mtx);
        callbacks.erase(handle.iterator);
        handle.iterator = typename std::list<std::function<T>>::iterator();
    }
}

template<typename T>
template<typename ...Args>
void EventHandlerList<T>::invoke(Args&& ... args)
{
    std::lock_guard lock(mtx);
    for (auto& handler : callbacks)
    {
        handler(std::forward<Args>(args)...);
    }
}

} // namespace fastsense::util
