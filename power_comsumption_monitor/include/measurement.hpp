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

#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <variant>

namespace power_consumption_monitor
{
using MeasurementResultType = std::map<std::string, std::variant<double, std::string>>;

class Measurement {
public:
    virtual ~Measurement() {}

    virtual void startMeasurement() = 0;
    virtual void stopMeasurement() = 0;
    virtual void clearMeasurement() = 0;
    virtual void updateMeasurement() = 0;
    virtual MeasurementResultType getMeasurementResult(double duration) = 0;

protected:
    Measurement() {}
};

} // power_consumption_monitor

#endif // MEASUREMENT_HPP_