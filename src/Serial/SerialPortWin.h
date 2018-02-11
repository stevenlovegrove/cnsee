/* Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

// TODO: Avoid this namespace pollution. Factor into cpp file.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <Setupapi.h>

#include <stdexcept>
#include <iostream>
#include <stdint.h>

#ifndef GUID_DEVCLASS_PORTS
DEFINE_GUID(GUID_DEVCLASS_PORTS, 0x4D36E978, 0xE325, 0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18);
#endif

typedef SSIZE_T ssize_t;

class SerialPort
{
public:
    static std::string FindDevice(int index = 0)
    {
        int ports_found = 0;
        std::string PortName = "COM1";

        HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);

        SP_DEVINFO_DATA spDevInfoData;

        ZeroMemory(&spDevInfoData, sizeof(SP_DEVINFO_DATA));
        spDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

        BYTE szBuf[2048];

        for (int wIndex = 0; ports_found <= index && SetupDiEnumDeviceInfo(hDevInfo, wIndex, &spDevInfoData); ++wIndex)
        {
            // Enumerate device properties
            //for (int p = 0; p < SPDRP_MAXIMUM_PROPERTY; ++p) {
            //    SetupDiGetDeviceRegistryProperty(hDevInfo, &spDevInfoData, p, 0L,
            //        szBuf, 2048, 0);
            //    std::cout << p << ": " << szBuf << std::endl;
            //}

            HKEY key = SetupDiOpenDevRegKey(hDevInfo, &spDevInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (key != INVALID_HANDLE_VALUE)
            {
                DWORD buflen = 2048;
                // Look up ports device name in the registry
                if (ERROR_SUCCESS == RegGetValue(key, NULL, "PortName", RRF_RT_REG_SZ, NULL, szBuf, &buflen)) {
                    PortName = std::string( (char*)szBuf);
                    ++ports_found;
                }
                RegCloseKey(key);
            }
        }

        return PortName;
    }

    SerialPort()
        : hComm(INVALID_HANDLE_VALUE)
    {
    }

    SerialPort(int index)
    {
        try {
            Open(FindDevice(index));
        } catch (std::runtime_error err) {
            std::cerr << "Unable to connect to Serial Port." << std::endl;
        }
    }

    SerialPort(std::string device_name)
        : hComm(INVALID_HANDLE_VALUE)
    {
        try {
            Open(device_name);
        } catch (std::runtime_error err) {
            std::cerr << "Unable to connect to Serial Port." << std::endl;
        }
    }


    ~SerialPort()
    {
        Close();
    }

    bool IsOpen()
    {
        return hComm != INVALID_HANDLE_VALUE;
    }

    void Open(std::string device_name)
    {
        if (IsOpen()) {
            Close();
        }

        // Open COM Port, non-overlaping mode (synchronous, blocking)
        hComm = CreateFile(device_name.c_str(), GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);
        if (hComm == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Unable to open com port");
        }

        // Read original COM Port state
        GetCommState(hComm, &original_state);

        // Initialise COM Port
        DCB dcb; 
        FillMemory(&dcb, sizeof(dcb), 0);
        dcb.DCBlength = sizeof(dcb);

        if (!BuildCommDCB("115200,n,8,1", &dcb)) {
            throw std::runtime_error("Unable to build Com Port parameters");
        }

        if (!SetCommState(hComm, &dcb)) {
            throw std::runtime_error("Unable to Set Com Port parameters");
        }

        Purge();
    }

    void Close()
    {
        if (IsOpen()) {
            if (!SetCommState(hComm, &original_state)) {
                throw std::runtime_error("Unable to restore Com Port parameters");
            }
            CloseHandle(hComm);
            hComm = 0;
        }
    }

    ssize_t Read(unsigned char* buffer, size_t num_bytes)
    {
        DWORD bytes_read;
        if (!ReadFile(hComm, buffer, num_bytes, &bytes_read, NULL)) {
            return -1;
        }
        return bytes_read;
    }

    bool WaitForRead(time_t seconds, long microseconds = 0)
    {
        throw std::runtime_error("Not implemented");
    }

    ssize_t Write(unsigned char* /*buffer*/, size_t /*num_bytes*/)
    {
        throw std::runtime_error("Not implemented");
    }

    void FinishWrite()
    {
        throw std::runtime_error("Not implemented");
    }

    ssize_t ReadCount()
    {   
        throw std::runtime_error("Not implemented");
    }

    int Purge(){
        return PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }

private:
    HANDLE hComm;
    DCB original_state;
};
