// Copyright 2021 Tier IV, In4.
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

#ifndef POWER_CONSUMPTION_MONITOR_CORE_HPP_
#define POWER_CONSUMPTION_MONITOR_CORE_HPP_

#include "configuration_loader.hpp"
#include "measurement.hpp"
#include "perf_measurement.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace power_consumption_monitor
{

constexpr bool DEBUG = false;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << X << std::endl;}}

class ConfigurationLoader;


class PowerConsumptionMonitor : public rclcpp::Node
{
public:
    PowerConsumptionMonitor();
    PowerConsumptionMonitor(rclcpp::NodeOptions& options);

private:
	void diagDataUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat);

	inline double get_now() { return nano_to_sec(get_clock()->now().nanoseconds()); }

	inline double nano_to_sec(double nano) { return nano / 1e9; }

    std::shared_ptr<ConfigurationLoader> config_ptr_;

	std::shared_ptr<Measurement> measurement_ptr_;

	std::shared_ptr<diagnostic_updater::Updater> updater_;

	std::string method_;

	std::string path_;
	
	double start_;
	
	double prev_;

	double period_;

};

} // power_consumption_monitor

#endif //POWER_CONSUMPTION_MONITOR_CORE_HPP_