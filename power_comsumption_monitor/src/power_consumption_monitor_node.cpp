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


static void signal_handler(int signum)
{
	printf("signal_handler: caught signal %d\n", signum);
	if (signum == SIGINT) {
		printf("--- SIGINT ---\n");
		exit(1);
	}
}

int main(int argc, char ** argv)
{
	/*
	const char* debug_env = std::getenv("DEBUG");
	bool debug_mode = false;
	if (debug_env != nullptr) {
		power_consumption_monitor::DEBUG = (std::atoi(debug_env) != 0);
	}
	*/
	std::cout << "DEBUG: " << power_consumption_monitor::DEBUG << std::endl;

	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		printf("## Error caught signal\n");
		return -1;
	}
	if (setuid(0) != 0) {
        std::cerr << "## Failed to setuid to root: " << strerror(errno) << std::endl;
    }
	rclcpp::init(argc, argv);
    auto node = std::make_shared<power_consumption_monitor::PowerConsumptionMonitor>();
	rclcpp::executors::SingleThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}