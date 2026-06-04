// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief Dynamic library loading utilities
 * @details
 * This file provides utilities for loading and managing dynamic libraries
 * in the Isaac Sim bridge. It includes classes for single and multiple
 * library loading with platform-specific handling.
 */
#pragma once
#include <carb/extras/Library.h>
#include <carb/logging/Log.h>

#include <memory>
#include <vector>
namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class LibraryLoader
 * @brief Single dynamic library loader
 * @details
 * Handles loading, symbol resolution, and unloading of a single dynamic library.
 * Provides platform-independent interface for library operations with automatic
 * cleanup on destruction.
 */
class LibraryLoader
{
public:
    std::string loadedLibraryFile; /**< Path to the loaded library file */
    carb::extras::LibraryHandle loadedLibrary = carb::extras::kInvalidLibraryHandle; /**< Handle to the loaded library
                                                                                      */

    /**
     * @brief Constructor for LibraryLoader
     * @details
     * Attempts to load the specified library with platform-specific naming conventions.
     * On Windows, appends .dll, on other platforms prepends lib and appends .so
     *
     * @param[in] library Base name of the library to load
     * @param[in] prefix Optional prefix to add to the library name
     * @param[in] test If true, suppresses error messages on load failure
     */
    LibraryLoader(std::string library, std::string prefix = "", bool test = false)
    {
        {
#ifdef _MSC_VER
            std::string libraryPath = prefix + library + ".dll";
#else
            std::string libraryPath = prefix + "lib" + library + ".so";
#endif
            loadedLibraryFile = libraryPath;
            // printf("Loading %s\n", libraryPath.c_str());
            // loadedLibrary = dlopen(libraryPath.c_str(), RTLD_NOW | RTLD_GLOBAL);
            loadedLibrary = carb::extras::loadLibrary(libraryPath.c_str(), carb::extras::fLibFlagNow);

            if (loadedLibrary == carb::extras::kInvalidLibraryHandle && !test)
            {

                printf("Could not load the dynamic library from %s. Error: %s\n", libraryPath.c_str(),
                       carb::extras::getLastLoadLibraryError().c_str());
            }
        }
    }

    /**
     * @brief Destructor
     * @details Ensures proper cleanup by unloading the library if it was loaded
     */
    ~LibraryLoader()
    {
        if (loadedLibrary)
        {
            // printf("Destructor for %s \n", loadedLibraryFile.c_str());
            // dlclose(loadedLibrary);
            carb::extras::unloadLibrary(loadedLibrary);
        }
        loadedLibrary = carb::extras::kInvalidLibraryHandle;
    }

    /**
     * @brief Gets a symbol from the loaded library
     * @details
     * Template function to retrieve a symbol of any type from the library
     *
     * @tparam T Type of the symbol to retrieve
     * @param[in] symbol Name of the symbol to retrieve
     * @return T The retrieved symbol cast to type T
     */
    template <typename T>
    T getSymbol(std::string symbol)
    {
        return carb::extras::getLibrarySymbol<T>(loadedLibrary, symbol.c_str());
    }

    /**
     * @brief Calls a symbol function with no arguments
     * @details
     * Template function to call a function symbol that takes no arguments
     *
     * @tparam T Return type of the function
     * @param[in] symbol Name of the function to call
     * @return T The return value from the function call, or nullptr if symbol not found
     */
    template <typename T>
    T callSymbol(std::string symbol)
    {
        using binding = T();
        void* loadedSymbol = getSymbol<void*>(symbol.c_str());

        if (!loadedSymbol)
        {
            printf("%s does not contain %s\n", loadedLibraryFile.c_str(), symbol.c_str());
            return nullptr;
        }
        binding* calledSymbol = reinterpret_cast<binding*>(loadedSymbol);

        return calledSymbol();
    }

    /**
     * @brief Calls a symbol function with arguments
     * @details
     * Template function to call a function symbol with variable number of arguments
     *
     * @tparam T Return type of the function
     * @tparam Arguments Types of the function arguments
     * @param[in] symbol Name of the function to call
     * @param[in] args Arguments to pass to the function
     * @return T The return value from the function call
     */
    template <typename T, typename... Arguments>
    T callSymbolWithArg(std::string symbol, Arguments... args)
    {
        using binding = T(Arguments...);
        void* loadedSymbol = getSymbol<void*>(symbol.c_str());

        if (!loadedSymbol)
        {
            printf("%s does not contain %s\n", loadedLibraryFile.c_str(), symbol.c_str());
        }

        binding* calledSymbol = reinterpret_cast<binding*>(loadedSymbol);

        return calledSymbol(args...);
    }

    /**
     * @brief Checks if the library is loaded
     * @return bool True if the library was loaded successfully
     */
    bool isValid()
    {
        // Return true if the library handle is not invalid
        return loadedLibrary != carb::extras::kInvalidLibraryHandle;
    }


private:
};

/**
 * @class MultiLibraryLoader
 * @brief Manager for multiple dynamic libraries
 * @details
 * Provides functionality to load and manage multiple dynamic libraries
 * simultaneously. Handles cleanup of all loaded libraries on destruction.
 */
class MultiLibraryLoader
{
public:
    /**
     * @brief Destructor
     * @details Cleans up all loaded libraries
     */
    ~MultiLibraryLoader()
    {
        m_loadedLibraries.clear();
    }

    /**
     * @brief Loads a new library
     * @details
     * Creates a new LibraryLoader instance for the specified library
     * and adds it to the managed collection
     *
     * @param[in] library Base name of the library to load
     * @param[in] prefix Optional prefix to add to the library name
     * @return std::shared_ptr<LibraryLoader> Pointer to the created loader
     */
    std::shared_ptr<LibraryLoader> loadLibrary(const std::string library, std::string prefix = "")
    {
        auto loadedLib = std::make_shared<LibraryLoader>(library, prefix);
        m_loadedLibraries.emplace_back(loadedLib);
        return loadedLib;
    }

private:
    std::vector<std::shared_ptr<LibraryLoader>> m_loadedLibraries; /**< Collection of managed library loaders */
};


}
}
}
