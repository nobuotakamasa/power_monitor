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
#include "measurement.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <variant>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include <unistd.h>
#include <limits.h>

static std::string hostname_modified() {
    char hostname[HOST_NAME_MAX + 1];
    if (RCUTILS_LIKELY(gethostname(hostname, sizeof(hostname)) == 0)) {
        std::cout << "Hostname: " << hostname << std::endl;
    } else {
        std::cerr << "Failed to get hostname" << std::endl;
    }
    std::string hostname_str(hostname);
    std::replace(hostname_str.begin(), hostname_str.end(), '-', '_');
	return hostname_str;
}

namespace power_consumption_monitor
{

// const
static constexpr char version[] = "v0.10";


extern std::string sysfs_path;
extern std::map<std::string, std::string> powercap_elements;

static std::string get_cpu_name() {
    std::ifstream infile("/proc/cpuinfo");
    std::string line;
    std::string cpu_name;
    while (std::getline(infile, line)) {
        if (line.find("model name") != std::string::npos) {
            // found
            cpu_name = line.substr(line.find(":") + 1);
            cpu_name.erase(0, cpu_name.find_first_not_of(' '));
            break;
        }
    }
    infile.close();

    return cpu_name;
}

/**
 * power consumption monitor node
 */
PowerConsumptionMonitor::PowerConsumptionMonitor()
: Node(
    "power_consumption_monitor_" + hostname_modified(), rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
) {
	// Get period parameter
    try {
			if (!this->has_parameter("period")) {
                this->declare_parameter<float>("period", 5.0f);
            }
            auto param_value = this->get_parameter("period").get_parameter_value();
            if (RCUTILS_UNLIKELY(param_value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)) {
                RCLCPP_ERROR(this->get_logger(), "Expected double type: %s", param_value);
                rclcpp::shutdown();
                exit(-1);
            } 
			period_ = this->get_parameter("period").as_double();

	} catch (const rclcpp::ParameterTypeException& e) {
			RCLCPP_ERROR(this->get_logger(), "Invalid parameters: %s", e.what());
			rclcpp::shutdown();
            exit(-1);
	} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
			rclcpp::shutdown();
            exit(-1);
	}

    // configurator
    try {
        config_ptr_ = std::make_shared<ConfigurationLoader>(this);
        auto config_data = config_ptr_->getConfigData();
        method_ = config_data.method;
        path_ = config_data.path;
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "ConfigurationLoader error: %s", e.what());
        rclcpp::shutdown();
        exit(-1);
    }

    // measurement class to instance
    measurement_ptr_ = std::make_shared<PerfMeasurement>(config_ptr_->getConfigData());
#if 0
    try {
        if (RCUTILS_UNLIKELY((measurement_ptr_ = MeasurementFactory::Factory(config_ptr_)) == nullptr)) {
			RCLCPP_ERROR(this->get_logger(), "Factory method: ", method_);
			rclcpp::shutdown();
            exit(-1);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Factory error: %s", e.what());
        rclcpp::shutdown();
        exit(-1);
    }
#endif
    // measurement start
    start_ = prev_ = get_now();
    measurement_ptr_->startMeasurement();

    // Diagnostic Update
    this->updater_ = std::make_shared<diagnostic_updater::Updater>(this, period_);
    // get host name
    char host_name[HOST_NAME_MAX + 1];
    gethostname(host_name, sizeof(host_name));
    // get processor name
    std::string cpu_name = get_cpu_name();
    std::string combined_name = cpu_name + "@" + host_name;
    this->updater_->setHardwareID(combined_name.c_str());
    this->updater_->add("", this, &PowerConsumptionMonitor::diagDataUpdate);

    RCLCPP_INFO(get_logger(), "\n\n--- %s start [diag period=%.2f] %s ---\n", method_.c_str(), period_, path_.c_str());
}

// update diagnostics data
void PowerConsumptionMonitor::diagDataUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat) {
    //RCLCPP_INFO(get_logger(), "[%.6f]--[%s]:%04d called", get_now(), __func__, __LINE__);
    using diagnostic_msgs::msg::DiagnosticStatus;

    auto diag_period = this->updater_->getPeriod().seconds();

    // Power data collection
    auto end = get_now();
    auto elapsed = end - start_;
    auto duration = end - prev_;
    
    measurement_ptr_->clearMeasurement();
    MeasurementResultType outputs = measurement_ptr_->getMeasurementResult(duration);
    measurement_ptr_->updateMeasurement();

    prev_ = end;
    stat.add("elapsed (Sec)", elapsed);
    stat.add("duration (Sec)", duration);

    for (auto& pair: outputs) {
        if (std::holds_alternative<double>(pair.second)) {
            double value = std::get<double>(pair.second);
            stat.add(pair.first, value);
        } else if (std::holds_alternative<std::string>(pair.second)) {
            const std::string& value = std::get<std::string>(pair.second);
            stat.add(pair.first, value);
        } else {
            // If you reach this else condition, it means your variant may be holding an unexpected type.
        }
    }

    int8_t level = DiagnosticStatus::OK;
    std::string msg = fmt::format("OK: diag period {} (Sec)", diag_period);
    stat.summary(level, msg);
}

}  // namespace power_consumption_monitor