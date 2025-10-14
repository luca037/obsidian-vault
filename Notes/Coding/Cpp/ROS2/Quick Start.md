# Source setup.zsh

To use ros2 we need to `source` a setup file. To avoid sourcing every time you open a shell, you can add

```shell
source /opt/ros/jazzy/setup.zsh # or .bash or whatever
```

to your `.zshrc` file.

# Creating a workspace

> A workspace is a directory containing ROS 2 packages. 

Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name `ws`.

Another best practice is to put any packages in your workspace into the src directory. 

```shell
mkdir -p ws/src
cd ws/src
```

# Creating a package

> A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. 

Structure of a package:

```
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```

A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.). 

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

To **create a package** inside your workspace directory:

```shell
ros2 pkg create \
--build-type ament_cmake \
--license Apache-2.0 cpp_pubsub
```

To **build a package**:

```shell
cd workspace_folder
colcon build
```

# Nodes

> Each node in ROS should be responsible for a *single*, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.


![[Pasted image 20251007111942.png]]

# Topics

> Topics act as a bus for nodes to exchange messages.

Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

![[Pasted image 20251007112916.png]]

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![[Pasted image 20251007113500.png]]

# Simple Publisher and Subscriber 

We start by writing the `minimal_publisher.cpp` inside the `ws/src/cpp_pubsub/src/` directory.

```cpp
#include <chrono>
#include <memory>
#include <string>

// Most common pieces in ROS2 system.
#include "rclcpp/rclcpp.hpp" 

// Include built-in message type (string)
// used to pubblish data.
#include "std_msgs/msg/string.hpp" 

using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() 
    : Node("minimal_publisher"), // Name of this node.
    count_(0) 
    {
        // Choose the name of the topic.
        std::string topic_name = "topic";
		
        // Required queue size to limit 
        // messages in the event of a backup.
        const int queue_size = 10; 

        // Create the pubblisher and the topic.
        publisher_ = 
	        this->create_publisher<std_msgs::msg::String>(
	            topic_name, queue_size
	        );

        // Callback used to publish data 
        // on the created topic.
        auto timer_callback =
            [this]() -> void { // Capture 'this' 
					           // by reference.
                // Create a message of type string.
                auto message = std_msgs::msg::String();

                // Store the message in 'data' filed.
                message.data = 
	                "Hello, world! " + 
	                std::to_string(this->count_++);

	            // Ensure that every published message is
	            // printed to the console.
                RCLCPP_INFO(
	                this->get_logger(),
	                 "Publishing: '%s'",
	                 message.data.c_str()
	            );

                // Pubblish the message.
                this->publisher_->publish(message);
            };

        // Init timer: causes the 'timer_callback 
        // to be executed every 500ms (twice a second).
        timer_ = this->create_wall_timer(
	        500ms, timer_callback
	    );
    }

private:
    // ROS2 timer: manages frequency of pubblished messages.
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS2 publisher: publishes string type messages.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Number of message pubblished.
    size_t count_;
};


int main(int argc, char * argv[]) {

    // Init ROS2 env.
    rclcpp::init(argc, argv);

    // Start procesing data from the node, 
    // including callbacks from the timer.
    rclcpp::spin(std::make_shared<MinimalPublisher>());

    // Close ROS2 env.
    rclcpp::shutdown();

    return 0;
}
```

Now we create `minimal_subscriber.cpp` in the same directory:

```cpp
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {

public:
    MinimalSubscriber()
        : Node("minimal_subscriber") // Name of the node.
    {
        
        // Callback to read data from the topic.
        auto topic_callback = // Receives the string 
						      // messages (same type 
						      // defined in the publisher).
            [this](std_msgs::msg::String::UniquePtr msg) {
                // Print 'msg' to the console.
                RCLCPP_INFO(
	                this->get_logger(),
	                "I heard: '%s'", 
	                msg->data.c_str()
	            );
            };

        // Makethe subscription to a topic.
        // (The same created by the publisher).
        std::string topic_name = "topic"; 
        // Max number of message in the buffer.
        const int queue_size = 10; 
        subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                topic_name, queue_size, topic_callback
            );
    }

private:
    // ROS2 subsciber: reads messages of type string.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    // Init ROS2 env.
    rclcpp::init(argc, argv);

    // Spin for the subsciber 
	//   -> means preparing to receive messages
    //      whenever they come.
    rclcpp::spin(std::make_shared<MinimalSubscriber>());

    // Close ROS2 env.
    rclcpp::shutdown();
    return 0;
}
```

Now we need to open the `package.xml` file and add:

```xml
<!-- Description of the project-->
<description>
	Examples of minimal publisher/subscriber using rclcpp
</description>
<maintainer email="luca@email.it">luca</maintainer>
<license>Apache-2.0</license>
```

and the dependencies:

```xml
  <export>
    <build_type>ament_cmake</build_type>
    <!-- Add dependencies -->
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>
  </export>
```

We need also to modify the `CMakeList.txt`. A minimal version is:

```cmake
cmake_minimum_required(VERSION 3.8)
project(pkg_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


# Added in our project:
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# Executable for the publisher.
add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Executable for the subsciber.
add_executable(listener src/minimal_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Install section -> so 'ros2 run' can find the executable:
install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

ament_package()
``` 

Finally we can **build and run the project**.

Enter in your `ws` directory and check for missing dependencies with:

```shell
rosdep install -i --from-path src --rosdistro jazzy -y
```

Compile with:

```shell
colcon build --packages-select cpp_pubsub
```

To execute, first we open a new terminal (still in `ws` directory) and we
run

```shell
. install/setup.zsh
```

In one terminal we run the publisher:

```shell
ros2 run cpp_pubsub talker
```

In the other one we run the listener:

```shell
ros2 run cpp_pubsub listener
```

... and we're done :)



