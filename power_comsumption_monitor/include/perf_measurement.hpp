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

#ifndef PERF_MEASUREMENT_HPP_
#define PERF_MEASUREMENT_HPP_

#include "configuration_loader.hpp"
#include "measurement.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <memory>
#include <variant>

#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace power_consumption_monitor
{

struct PerfEvent {
	int event_no;
	std::string unit;
	double scale;
	int fd;
};

class ConfigurationData;


class PerfMeasurement: public Measurement {
public:
    PerfMeasurement(const ConfigurationData& config_data);

    MeasurementResultType getMeasurementResult(double duration);

    void startMeasurement();

    void stopMeasurement() {};

    void clearMeasurement() {};
    
    void updateMeasurement() {};
    

private:
    int perf_event_open_(
        struct perf_event_attr *hw_event,
        pid_t pid,
        int cpu,
        int group_fd,
        unsigned long flags
    );
    
    int get_perf_info_(std::string file_path);
    
    template <typename T>
    T read_value_(const std::filesystem::path& filepath);
    
    int perf_event_open_helper_(int type, int event_no);
    
    std::map<std::string, PerfEvent> create_perf_event_map_(int type, const std::string& base_path);

    const ConfigurationData& config_data_;

    int perf_type_;
    
    std::map<std::string, PerfEvent> perf_events_;

};

} // power_consumption_monitor

#endif // PERF_MEASUREMENT_HPP_