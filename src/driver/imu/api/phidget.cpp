/*
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

#include "phidget.h"
#include <stdexcept>
#include <util/logging/logger.h>

using namespace fastsense::driver;
using namespace fastsense::util::logging;

Phidget::Phidget() : handle_{} {}

Phidget::~Phidget()
{
    close(); // TODO segfaults, why?
    CPhidget_delete(handle_);
}

void Phidget::registerHandlers()
{
    CPhidget_set_OnAttach_Handler(handle_, &Phidget::attach_handler, this);
    CPhidget_set_OnDetach_Handler(handle_, &Phidget::detach_handler, this);
    CPhidget_set_OnError_Handler(handle_, &Phidget::error_handler, this);
}

void Phidget::init(CPhidgetHandle handle)
{
    handle_ = handle;
}

void Phidget::openAndWaitForAttachment(int serial_number, int timeout)
{
    int ret = CPhidget_open(handle_, serial_number);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Open error: " + Phidget::getErrorDescription(ret));
    }

    ret = CPhidget_waitForAttachment(handle_, timeout);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Attachment error: " + Phidget::getErrorDescription(ret));
    }
}

int Phidget::close()
{
    return CPhidget_close(handle_);
}

std::string Phidget::getDeviceType()
{
    char a[1000];
    const char* deviceptr = a;
    CPhidget_getDeviceType(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget::getDeviceName()
{
    char a[1000];
    const char* deviceptr = a;
    CPhidget_getDeviceName(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget::getDeviceLabel()
{
    char a[1000];
    const char* deviceptr = a;
    CPhidget_getDeviceType(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget::getLibraryVersion()
{
    char a[1000];
    const char* deviceptr = a;
    CPhidget_getLibraryVersion(&deviceptr);
    return std::string(deviceptr);
}

int Phidget::getDeviceSerialNumber()
{
    int sernum;
    CPhidget_getSerialNumber(handle_, &sernum);
    return sernum;
}

int Phidget::getDeviceVersion()
{
    int version;
    CPhidget_getDeviceVersion(handle_, &version);
    return version;
}

std::string Phidget::getErrorDescription(int errorCode)
{
    char a[1000];
    const char* errorPtr = a;
    CPhidget_getErrorDescription(errorCode, &errorPtr);
    return std::string(errorPtr);
}

void Phidget::attach_handler()
{
    Logger::info("Phidget attached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget::detach_handler()
{
    Logger::info("Phidget detached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget::error_handler(int error)
{
    Logger::error("Phidget error [%d]: %s\n", error,
           getErrorDescription(error).c_str());
}

int Phidget::attach_handler(CPhidgetHandle /* handle */, void* userptr)
{
    ((Phidget*)userptr)->attach_handler();
    return 0;
}

int Phidget::detach_handler(CPhidgetHandle /* handle */, void* userptr)
{
    ((Phidget*)userptr)->detach_handler();
    return 0;
}

int Phidget::error_handler(CPhidgetHandle /* handle */, void* userptr,
                           int ErrorCode, const char* /* unknown */)
{
    ((Phidget*)userptr)->error_handler(ErrorCode);
    return 0;
}
