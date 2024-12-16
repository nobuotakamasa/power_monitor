// Copyright 2024 Tier IV, Inc.
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

#include "power_consumption_monitor_core.hpp"
#include "configuration_loader.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <cerrno>
#include <cstring>
#include <string>
#include <map>
#include <chrono>
#include <variant>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace power_consumption_monitor
{
    
ConfigurationLoader::ConfigurationLoader(PowerConsumptionMonitor* node)
: node_(node) {
    if (!load_config_from_yaml_()) {
        throw std::runtime_error("Error: load_config_from_yaml_");    
    }
    //std::cout << config_data_.method << ": " << config_data_.path << std::endl;
}

// load config.yaml
bool ConfigurationLoader::load_config_from_yaml_() {
    // get method
    if (node_->has_parameter("method")) {
        node_->get_parameter("method", config_data_.method);
    } else {
        return false;
    }
    // get path
    std::string path("powercap_path");
    if (config_data_.method == "perf") {
       path = "perf_path"; 
    }
    if (node_->has_parameter(path.c_str())) {
        std::string path_name;
        node_->get_parameter(path.c_str(), path_name);
        if (path_name[path_name.length() - 1] == '/') {
            path_name[path_name.length() - 1] = '\0';
        }
        config_data_.path = path_name;
    } else {
        return false;
    }

    if (config_data_.method == "perf") {
        return true;
    }
    
    // get entry
    static const std::unordered_set<std::string> allowed_values = {"int", "str", "diff"};
    auto entry_names = node_->list_parameters({"powercap_entry"}, 2).names;
    for (const auto &entry_name : entry_names) {
        std::string value;
        node_->get_parameter(entry_name, value);
        if (allowed_values.find(value) == allowed_values.end()) {
            return false;
        }
        std::string prefix = "powercap_entry.";
        std::string name = entry_name;
        if (name.rfind(prefix, 0) == 0) {
            name.erase(0, prefix.length());
        }
        config_data_.entries.emplace(name, value);
    }
    return true;
}

ConfigurationData & ConfigurationLoader::getConfigData() {
    return config_data_;
}

} // power_consumconfig_dataor