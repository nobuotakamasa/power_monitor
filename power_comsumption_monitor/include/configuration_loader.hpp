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

#ifndef CONFIGURATION_LOADER_HPP_
#define CONFIGURATION_LOADER_HPP_

#include "power_consumption_monitor_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <memory>

namespace power_consumption_monitor
{
/***
  ros__parameters:
    period: 5.0
    method: perf
    path: /sys/devices/power
    #path: /sys/class/powercap/intel-rapl
    entry:
      energy_uj: diff
      #max_energy_range_uj: int
      constraint_0_name: str
      constraint_0_max_power_uw: int
      constraint_0_power_limit_uw: int
      constraint_0_time_window_us: int
      constraint_1_name: str
      constraint_1_power_limit_uw: int
      constraint_1_time_window_us: int
***/

struct ConfigurationData {
    std::string method;
    std::string path;
    std::map<std::string, std::string> entries;
};

class PowerConsumptionMonitor;


class ConfigurationLoader {
public:
    ConfigurationLoader(PowerConsumptionMonitor* node);

    ConfigurationData & getConfigData();

private:
    bool load_config_from_yaml_();

    ConfigurationData config_data_;

    PowerConsumptionMonitor *node_;
};

} // power_consumption_monitor

#endif // CONFIGURATION_LOADER_HPP_