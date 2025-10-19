# Exercise: custom message

We have a roomba like robot that needs to clean the rooms of a building.

| **Component**                      | **Description**                                                                                                                                                  |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Custom Message (Msg)               | Create a ROS2 message type containing: <br> - Room ID<br> - Room name<br> - Battery level                                                                        |
| Robot Node (Publisher)             | - Publishes robot data to a topic at **5Hz** <br> - Data includes: room ID, room name, battery level <br> - Topic name: `/charge_status`                         |
| Charging Station Node (Subscriber) | - Subscribes to the `/charge_status` topic at **5Hz** <br> - Prints received messages (room ID, name, battery) to the terminal                                   |
| Room Details                       | Use the provided lab names and IDs: <br> - 1: Robot Vision Lab <br> - 2: SSL Lab <br> - 3: Neurorobotics Lab <br> - 4: IAS-Lab <br> - 5: Autonomous Robotics Lab |
| Topic Frequency                    | Both publisher and subscriber operate at **5Hz**                                                                                                                 |
| Visual/Graph Structure             | System follows the structure: <br> `/robot` → (publishes) → `/charge_status` → (received by) → `/charging_station`                                               |

The logic regarding battery drain, the choice of the room to clean and the cleaning time are up to you.

What i've done:

- Battery drains 1% every second (start from 100%).
- Room to clean selected at random.
- Time to clean a room: 3 seconds.

### Solution

File: `ros_ws/src/vac_interfaces/msg/Status.msg`

```
int64 id
string name
int64 battery
```

File: `ros_ws/src/vacuum/src/robot.cpp`:

```cpp
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp" 
#include "vac_interfaces/msg/status.hpp"

using namespace std::chrono_literals;

// Rooms of computer vision lab.
const std::vector<std::string> ROOMS = {
    "robot_vision_lab",
    "ssl_lab",
    "neurorob_lab",
    "ias_lab",
    "autonomous_rob_lab"
};


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() 
    : Node("minimal_publisher"),
      counter_{0},
      battery_{100},
      room_idx_{0}
    {
        publisher_ = 
	        this->create_publisher
	        <vac_interfaces::msg::Status>(
	            "topic", 10 
	        );

        auto timer_callback =
            [this]() -> void {
                // Update battery, if necessary.
                if (!(++counter_ % 5) && battery_ > 0) 
	                battery_ -= 1;

                // Change room if you have finished 
                // cleaning the current one.
                bool chg_room = (!(counter_ % 15))? 1 : 0;

                // If you have enough battery and 
                // you have finished cleaning 
                // current room, then change room.
                if (battery_ && chg_room) {
                    size_t rnd;
                    while(
	                    (rnd = rand() % ROOMS.size())
	                    == room_idx_
                    );
                    room_idx_ = rnd;
                }

                RCLCPP_INFO(
                    this->get_logger(), 
                    "Sending status...\n\t room_idx_=%lu",
                    room_idx_
                );

                // Create the message.
                auto message = vac_interfaces::msg::Status();
                message.id = room_idx_ + 1;
                message.name = ROOMS[room_idx_];
                message.battery = battery_;

                this->publisher_->publish(message);
            };

        timer_ = this->create_wall_timer(
	        0.2s, // Frequency
	        timer_callback
	    );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher
    <vac_interfaces::msg::Status>::SharedPtr publisher_;
    size_t counter_;
    long int battery_;
    size_t room_idx_;
};


int main(int argc, char * argv[]) {
    srand(time(0));
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

File: `ros_ws/src/vacuum/src/charger.cpp`:

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vac_interfaces/msg/status.hpp"

class MinimalSubscriber : public rclcpp::Node {

public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        
        auto topic_callback = 
            [this]
            (vac_interfaces::msg::Status::UniquePtr msg) {
                RCLCPP_INFO(
	                this->get_logger(),
	                "\n\tReceived->  "
	                "id: %ld, name: %s, battery: %ld", 
                    msg->id,
	                msg->name.c_str(),
                    msg->battery
	            );
            };

        subscription_ =
            this->create_subscription
            <vac_interfaces::msg::Status>(
                "topic", 10, topic_callback
            );
    }

private:
    rclcpp::Subscription
    <vac_interfaces::msg::Status>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```