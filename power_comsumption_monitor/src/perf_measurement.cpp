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

#include "perf_measurement.hpp"

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

PerfMeasurement::PerfMeasurement(const ConfigurationData& config_data)
    : config_data_(config_data) {}

// collection perf event
int PerfMeasurement::perf_event_open_(
    struct perf_event_attr *hw_event,
    pid_t pid,
    int cpu,
    int group_fd,
    unsigned long flags
) {
    return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

int PerfMeasurement::get_perf_info_(std::string file_path) {
    std::ifstream file(file_path);
    std::string line;
    if (RCUTILS_LIKELY(bool(file >> line))) {
        try {
            int value = std::stoi(line);
            if (DEBUG) {std::cout << "Read value: " << value << std::endl;}
            return value;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: could not convert to int " << file_path << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Out of range: value cannot fit in an int " << file_path << std::endl;
        }
    } else {
        std::cerr << "Failed to read from file: " << file_path << std::endl;
    }
    exit(-1);

    /***
    int data;
    if (!file >> data) {
        perror("");
        std::cerr << "## Warning: Failed to read " << file_path << std::endl;
        exit(-1);
    }
    DB(file_path + " " + std::to_string(data))
    return data;
    ***/
}

// get file
template <typename T>
T PerfMeasurement::read_value_(const std::filesystem::path& filepath) {
    DB(filepath)
    std::ifstream file(filepath);
    if (RCUTILS_UNLIKELY(!file)) {
        throw std::runtime_error("## Could not open file for reading.");
    }

    T value;
    file >> value;

    if (RCUTILS_UNLIKELY(file.fail())) {
        throw std::runtime_error("Error reading the value from the file.");
    }
    DB(value)

    return value;
}

int PerfMeasurement::perf_event_open_helper_(int type, int event_no) {
    pid_t pid = -1;
    int cpu = 0;
    int group_fd = -1;
    unsigned long flags = 0;

    struct perf_event_attr pe_attr {};
    pe_attr.size = sizeof(pe_attr);
    pe_attr.type = type;
    pe_attr.config = event_no;
    pe_attr.disabled = 0;
    pe_attr.inherit = 1;
    pe_attr.exclude_kernel = 0;
    pe_attr.exclude_hv = 0;
    int fd = perf_event_open_(&pe_attr, pid, cpu, group_fd, flags);
    if (RCUTILS_UNLIKELY(fd == -1)) {
        perror("perf_event_open_");
        exit(-1);
    }
    // counter clear and restart
    ioctl(fd, PERF_EVENT_IOC_RESET, 0);
    ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
    return fd;
}

std::map<std::string, PerfEvent> PerfMeasurement::create_perf_event_map_(int type, const std::string& base_path) {
    std::filesystem::path path(base_path);
    std::map<std::string, PerfEvent> perf_events;

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        const std::filesystem::path& filepath = entry.path();
        std::string filename = filepath.filename().string();
        DB("")

        std::string prefix = "energy-";
        size_t prefix_length = prefix.length();
        if(filename.compare(0, prefix_length, prefix) == 0) {
            size_t last_dot_pos = filename.find_last_of('.');
            std::string key;

            // extract key from file name
            if(last_dot_pos != std::string::npos) {
                key = filename.substr(prefix_length, last_dot_pos - prefix_length);
            } else {
                key = filename.substr(prefix_length);
            }

            if (perf_events.find(key) == perf_events.end()) {
                // create new entry
                perf_events[key] = PerfEvent();
            }
            DB(key)
            
            DB(filepath)
            if (last_dot_pos == std::string::npos) {
                // "." is not present, the perf event number file ex. energy-core
                std::ifstream event_file(filepath);
                std::string event_no_str;
                if (!event_file.is_open() or !(event_file >> event_no_str)) {
                    perror(filepath.c_str());
                    continue;
                }
                std::stringstream ss(event_no_str);
                // skip "event_no=" ex. event_no=0x01
                std::string dummy;
                std::getline(ss, dummy, '=');
                // hex
                unsigned int event_no;
                ss >> std::hex >> event_no;
                perf_events[key].event_no = event_no;
                DB(event_no_str + " " + std::to_string(type))
                perf_events[key].fd = perf_event_open_helper_(type, perf_events[key].event_no);
                DB((key + " " + std::to_string(perf_events[key].fd)))
            } else {
                std::string extension = filename.substr(last_dot_pos);
                if (extension == ".scale") {
                    // ex. energy-core.scale
                    std::ifstream scale_file(filepath);
                    if (!scale_file.is_open() or !(scale_file >> perf_events[key].scale)) {
                        perror(filepath.c_str());
                    }
                    DB(perf_events[key].scale)
                } else if (extension == ".unit") {
                    // ex. energy-core.unit
                    std::ifstream unit_file(filepath);
                    if (!unit_file.is_open() or !(unit_file >> perf_events[key].unit)) {
                        perror(filepath.c_str());
                    }
                    DB(perf_events[key].unit)
                }
            }
        }
    }
    
    return perf_events;
}

void PerfMeasurement::startMeasurement() {
    DB(config_data_.path)
    perf_type_ = get_perf_info_(config_data_.path + "/type");
    DB(perf_type_)
    perf_events_ = create_perf_event_map_(perf_type_, config_data_.path + "/events");
    DB("perf_events_")
}

MeasurementResultType PerfMeasurement::getMeasurementResult(double duration) {
    DB("")
    std::map<std::string, std::variant<double, std::string>> items;
    for (auto& pair: perf_events_) {
        uint64_t count;
        DB((pair.first + " " + std::to_string(pair.second.fd)))
        ioctl(pair.second.fd, PERF_EVENT_IOC_DISABLE, 0);

        if (RCUTILS_UNLIKELY(read(pair.second.fd, &count, sizeof(uint64_t)) == -1)) {
            std::cerr << "Could not read event count: " << pair.first;
            close(pair.second.fd);
            exit(-1);
        }

        ioctl(pair.second.fd, PERF_EVENT_IOC_RESET, 0);
        ioctl(pair.second.fd, PERF_EVENT_IOC_ENABLE, 0);

        DB(count)
        DB(pair.second.scale)
        DB(pair.second.unit)
        double consumption_energy = count * pair.second.scale;
        double watt = consumption_energy / duration;
        items.emplace(pair.first + " energy consumption (Joule)", consumption_energy);
        items.emplace(pair.first + " average power consumption (Watt)", watt);
    }

    DB("")
    return items;
}

} // power_consumption_monitor