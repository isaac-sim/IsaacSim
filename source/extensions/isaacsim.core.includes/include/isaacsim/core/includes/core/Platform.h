// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
 
#include <cstdint>
#include <string>
#include <vector>

#ifndef _WIN32
#    include <sys/stat.h>
#    include <sys/types.h>

#    include <unistd.h>
#endif

// system functions
double GetSeconds();
void Sleep(double seconds);

// helper function to get exe path
std::string GetExePath();
std::string GetWorkingDirectory();


inline std::string GetFilePathByPlatform(const char* path)
{
    std::string newPath(path);
    return newPath;
}

// shows a file open dialog
// std::string FileOpenDialog(const char *filter = "All Files (*.*)\0*.*\0");

// pulls out an option in the form option=value, must have no spaces
template <typename T>
bool GetCmdLineArg(const char* arg, T& out, int argc, char* argv[])
{
    // iterate over all the arguments
    for (int i = 0; i < argc; ++i)
    {
        const char* s1 = arg;
        const char* s2 = argv[i];

        while (*s1 && *s2 && *s1 == *s2)
        {
            ++s1;
            ++s2;
        }

        // we've found the flag we're looking for
        if (*s1 == 0 && *s2 == '=')
        {
            ++s2;

            // build a string stream and output
            std::istringstream is(s2);
            if (is >> out)
            {
                return true;
            }
            else
                return false;
        }
    }

    return false;
}
// return the full path to a file
std::string ExpandPath(const char* path);
// takes a full file path and returns just the folder (with trailing slash)
std::string StripFilename(const char* path);
// strips the path from a file name
std::string StripPath(const char* path);
// strips the extension from a path
std::string StripExtension(const char* path);
// returns the file extension (excluding period)
std::string GetExtension(const char* path);
// normalize path
std::string NormalizePath(const char* path);

// loads a file to a text string
std::string LoadFileToString(const char* filename);
// loads a file to a binary buffer (free using delete[])
uint8_t* LoadFileToBuffer(const char* filename, uint32_t* sizeRead = nullptr);
// save whole string to a file
bool SaveStringToFile(const char* filename, const char* s);

bool FileMove(const char* src, const char* dest);
bool FileScan(const char* pattern, std::vector<std::string>& files);

// file system stuff
const uint32_t kMaxPathLength = 2048;

#ifdef WIN32

// defined these explicitly because don't want to include windowsx.h
#    define GET_WPARAM(wp, lp) (wp)
#    define GET_LPARAM(wp, lp) (lp)

#    define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#    define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))

#    define vsnprintf _vsnprintf
#    define snprintf _snprintf
#    define vsnwprintf _vsnwprintf

#    if _MSC_VER >= 1400 // vc8.0 use new secure
#        define snwprintf _snwprintf_s
#    else
#        define snwprintf _snwprintf
#    endif // _MSC_VER

#endif // WIN32


inline std::string ExpandPath(const char* p)
{
    return p;
}

inline std::string DataPath(const char* p)
{
    return ExpandPath((std::string("Data/") + p).c_str());
}
