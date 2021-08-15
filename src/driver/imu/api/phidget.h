/**
 * @file phidget.h
 * Copyright (c) 2009, Tully Foote
 * Copyright (c) 2011, Ivan Dryanovski
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PHIDGETS_API_PHIDGET_H
#define PHIDGETS_API_PHIDGET_H

#include <phidget21.h>
#include <string>

namespace fastsense
{
namespace driver
{

class Phidget
{
public:
    /**
     * @brief Construct a new Phidget object with empty handle
     */
    Phidget();

    /**
     * @brief Destroy the Phidget object: Handles closing of connection
     */
    virtual ~Phidget();

    /**
     * @brief delete assignment operator because of pointer member variable
     *
     * @return Phidget& other Phidget
     */
    Phidget& operator=(const Phidget& other) = delete;

    /**
     * @brief delete move assignment operator because of pointer member variable
     *
     * @return Phidget&& other Phidget (rval)
     */
    Phidget& operator=(const Phidget&& other) = delete;

    /**
     * @brief delete copy constructor because of pointer member variable
     * @param Phidget& other Phidget
     */
    Phidget(Phidget& other) = delete;

    /**
     * @brief delete move constructor because of pointer member variable
     * @param Phidget&& other Phidget (rval)
     */
    Phidget(Phidget&& other) = delete;

    /**@brief Open a connection to a Phidget
     * @param serial_number The serial number of the phidget to which to attach
     * (-1 will connect to any)
     * @param timeout Milliseconds to wait before timing out
     */
    void openAndWaitForAttachment(int serial_number, int timeout);

    /**@brief Close the connection to the phidget */
    int close();

    /** @brief Get the device type string */
    std::string getDeviceType();

    /** @brief Get the device name string */
    std::string getDeviceName();

    /** @brief Get the device label string */
    std::string getDeviceLabel();

    /** @brief Get the library version string */
    std::string getLibraryVersion();

    /** @brief Get the Phidget's serial number */
    int getDeviceSerialNumber();

    /** @brief Get the Phidget's version */
    int getDeviceVersion();

    /** @brief Lookup the string for a CPhidget Error Code
     *@param errorCode The error code returned from the CPhidget API */
    static std::string getErrorDescription(int errorCode);

protected:
    void init(CPhidgetHandle handle);

    void registerHandlers();
    virtual void attach_handler();
    virtual void detach_handler();
    virtual void error_handler(int error);

private:
    /// device handler
    CPhidgetHandle handle_;

    static int attach_handler(CPhidgetHandle handle, void* userptr);
    static int detach_handler(CPhidgetHandle handle, void* userptr);
    static int error_handler(CPhidgetHandle handle, void* userptr, int ErrorCode,
                             const char* unknown);
};

}  // namespace driver
}  // namespace phidgets

#endif  // PHIDGETS_API_PHIDGET_H
