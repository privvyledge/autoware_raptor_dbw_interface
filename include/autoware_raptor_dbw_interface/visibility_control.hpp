// Copyright 2024 Boluwatife Olabiran
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_RAPTOR_DBW_INTERFACE__VISIBILITY_CONTROL_HPP_
#define AUTOWARE_RAPTOR_DBW_INTERFACE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(AUTOWARE_RAPTOR_DBW_INTERFACE_BUILDING_DLL) || defined(AUTOWARE_RAPTOR_DBW_INTERFACE_EXPORTS)
    #define AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC __declspec(dllexport)
    #define AUTOWARE_RAPTOR_DBW_INTERFACE_LOCAL
  #else  // defined(AUTOWARE_RAPTOR_DBW_INTERFACE_BUILDING_DLL) || defined(AUTOWARE_RAPTOR_DBW_INTERFACE_EXPORTS)
    #define AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC __declspec(dllimport)
    #define AUTOWARE_RAPTOR_DBW_INTERFACE_LOCAL
  #endif  // defined(AUTOWARE_RAPTOR_DBW_INTERFACE_BUILDING_DLL) || defined(AUTOWARE_RAPTOR_DBW_INTERFACE_EXPORTS)
#elif defined(__linux__)
  #define AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC __attribute__((visibility("default")))
  #define AUTOWARE_RAPTOR_DBW_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC __attribute__((visibility("default")))
  #define AUTOWARE_RAPTOR_DBW_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // AUTOWARE_RAPTOR_DBW_INTERFACE__VISIBILITY_CONTROL_HPP_
