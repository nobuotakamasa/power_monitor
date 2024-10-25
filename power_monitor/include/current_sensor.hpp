#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>
#include <signal.h>


#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <cstdio>

#define __DEBUG__ 0

#define TOPIC_PREFIX "/ecu"
static bool use_timestamp_;

static inline int64_t toNanoseconds(const struct timeval& tv)
{
    return static_cast<int64_t>(tv.tv_sec) * 1000000000LL + static_cast<int64_t>(tv.tv_usec) * 1000LL;
}

static inline uint32_t to_msec(const struct timeval& tv)
{
    return static_cast<uint32_t>(tv.tv_sec) * 1000UL + static_cast<uint32_t>(tv.tv_usec) / 1000UL;
}

static inline builtin_interfaces::msg::Time toROSTime(const struct timeval& tv)
{
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = tv.tv_sec;
    ros_time.nanosec = tv.tv_usec * 1000;
    return ros_time;
}
//get arrival time of a CAN frame
static inline timeval get_arrival_time(struct msghdr& msg){
    struct timeval tv;
    for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg; cmsg = CMSG_NXTHDR(&msg, cmsg))
    {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP)
        {
            std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
        }
    }
    return tv;
}


class SensorPublisher {
protected:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr timestamp_publisher_;
    void publishData() {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = values_;
        publisher_->publish(msg);
        if(use_timestamp_) {
            auto timestamp_msg = std_msgs::msg::Int64MultiArray();
            timestamp_msg.data = timestamps_;
            timestamp_publisher_->publish(timestamp_msg);
        }
    }

    std::vector<float> values_;
    std::vector<int64_t> timestamps_;
    void refill() {
        std::fill(values_.begin(), values_.end(), std::nanf(""));
        if(use_timestamp_) {
            std::fill(timestamps_.begin(), timestamps_.end(), 0);
        }
    }
    void resize(int size) {
        values_.resize(size, std::nanf(""));
        if(use_timestamp_) {
            timestamps_.resize(size);
        }
    }
    SensorPublisher(rclcpp::Node* node) {
        //node->declare_parameter<bool>("use_timestamp", true);
        //use_timestamp_ = node->get_parameter("use_timestamp").as_bool();
    }
public:
    virtual bool publish(const struct can_frame& frame, struct msghdr& msg) = 0;
    virtual ~SensorPublisher(){}
};

class VoltageSensorPublisher : public SensorPublisher {
    int can_id_;
    double voltage_unit_;
public:
    VoltageSensorPublisher(rclcpp::Node* node) : SensorPublisher(node){
        node->declare_parameter<int>("voltage_can_id", 0);
        can_id_ = node->get_parameter("voltage_can_id").as_int();

        node->declare_parameter<double>("voltage_unit", 0.001);
        voltage_unit_ = node->get_parameter("voltage_unit").as_double();

        publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(TOPIC_PREFIX"/voltages/values", 10);
        if(use_timestamp_) {
            timestamp_publisher_ = node->create_publisher<std_msgs::msg::Int64MultiArray>(TOPIC_PREFIX"/voltages/timestamps", 10);
        }
        resize(2);
    }

    bool publish(const struct can_frame& frame, struct msghdr& msg) override {
        int canid = frame.can_id;// & 0xffff;//TODO: hack
        //if CAN ID is voltage sensor
        if (canid == can_id_)
        {
            int16_t v1 = 0;
            std::memcpy(&v1, frame.data, sizeof(v1));
            v1 = ntohs(v1);

            int16_t v2 = 0;
            std::memcpy(&v2, frame.data + 2, sizeof(v2));
            v2 = ntohs(v2);

            float v1_float = static_cast<float>(v1) * voltage_unit_;
            float v2_float = static_cast<float>(v2) * voltage_unit_;

            values_[0] = v1_float;
            values_[1] = v2_float;
            if(use_timestamp_) {
                struct timeval tv;
                tv = get_arrival_time(msg);
                timestamps_[0] = toNanoseconds(tv);
                timestamps_[1] = toNanoseconds(tv);
            }
            publishData();
            refill();
            return true;
        }
        return false;
    }
};

class CurrentSensorPublisher : public SensorPublisher {
    int start_can_id_;
    int num_sensors_;
public:
    CurrentSensorPublisher(rclcpp::Node* node) : SensorPublisher(node) {
        node->declare_parameter<int>("start_can_id", 0x3C2);
        node->declare_parameter<int>("num_sensors", 9);
        start_can_id_ = node->get_parameter("start_can_id").as_int();
        num_sensors_ = node->get_parameter("num_sensors").as_int();


        publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(TOPIC_PREFIX"/currents/values", 10);
        if(use_timestamp_) {
            timestamp_publisher_ = node->create_publisher<std_msgs::msg::Int64MultiArray>(TOPIC_PREFIX"/currents/timestamps", 10);
        }
        resize(num_sensors_);
    }
    bool publish(const struct can_frame& frame, struct msghdr& msg) override {
        bool result = false;
        //if CAN ID is current sensor
        int sensor_id = frame.can_id - start_can_id_;
        if (0 <= sensor_id && sensor_id < num_sensors_)
        {
            int32_t value = 0;
            std::memcpy(&value, frame.data, sizeof(value));
            value = ntohl(value) - 0x80000000L;

            if (!std::isnan(values_[sensor_id])) {
                publishData();
                refill();
                result = true;
			}
            values_[sensor_id] = static_cast<float>(value) / 1000.0;
            if(use_timestamp_) {
                struct timeval tv;
                tv = get_arrival_time(msg);
                timestamps_[sensor_id] = toNanoseconds(tv);
            }
        }
        return result;
    }
};

class CanSensorNode: public rclcpp::Node {
public:
    CanSensorNode(std::string node_name) : Node(node_name) {
        this->declare_parameter<std::string>("can_channel", "can0");
        can_channel_ = this->get_parameter("can_channel").as_string();
        socket_ = -1;
    }

protected:
    std::string can_channel_;
    int socket_;
    void closeCanInterface() {
        if( RCUTILS_UNLIKELY(close(socket_) < 0 )) {
            perror("close");
        }
    }
    void openCanInterface()
    {
        
        if (RCUTILS_UNLIKELY((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0))
        {
            perror("Socket");
            exit(1);
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, can_channel_.c_str());
        ioctl(socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (RCUTILS_UNLIKELY(bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0))
        {
            perror("Bind");
            exit(1);
        }

        const int timestamp_on = 1;
        if (RCUTILS_UNLIKELY(setsockopt(socket_, SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on)) < 0))
        {
            perror("setsockopt SO_TIMESTAMP");
            exit(1);
        }
    }
};

class PowerSensorNode : public CanSensorNode
{
    SensorPublisher* voltage_sensor_;
    SensorPublisher* current_sensor_;
public:
    PowerSensorNode()
        : CanSensorNode("current_sensor_node")
    {
        openCanInterface();
        voltage_sensor_ = new VoltageSensorPublisher(this);
        current_sensor_ = new CurrentSensorPublisher(this);
    }
    ~PowerSensorNode(){
        closeCanInterface();
    }

    void pollingLoop()
    {
        struct can_frame frame;
        struct msghdr msg;
        struct iovec iov;
        struct sockaddr_can addr;
        char control[1024];//TODO: magic number
        bool published = false;

        while (rclcpp::ok())
        {
            while (!published)
            {
                iov.iov_base = &frame;
                iov.iov_len = sizeof(frame);
                msg.msg_name = &addr;
                msg.msg_namelen = sizeof(addr);
                msg.msg_iov = &iov;
                msg.msg_iovlen = 1;
                msg.msg_control = &control;
                msg.msg_controllen = sizeof(control);
                msg.msg_flags = 0;

                int nbytes = recvmsg(socket_, &msg, MSG_WAITALL);
                if (RCUTILS_UNLIKELY(nbytes < 0))
                {
                    perror("Read");
                    break;
                }
                published |= voltage_sensor_->publish(frame, msg);
                published |= current_sensor_->publish(frame, msg);
            }
            published = false;
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

};

