# Creating custom messages

We will see how to create custom messages and we will test them using the publisher/subscriber package that we have created in the Quick Start tutorial.

### Create new package

In this package we will store the custom message. 

```shell
ros2 pkg create \
--build-type  ament_cmake \
--license Apache-2.0 tutorial_interfaces
``` 

Run the command above in the workspace directory in which you also have the publisher/subscriber package.

Create the directory `msg` inside the package that we have just created. Inside the directory create the file `Num.msg` and write in it

```
int64 num
```

Our custom message will have only a single field called `num` whose type is `int64`.

Finally we need to modify the `CMakeList.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
)
```

The name of the interface that will be generated will start with the name of the package.

Also we modify `package.xml`:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Build the package

In the workspace directory run:

```shell
colcon build --packages-select tutorial_interfaces
```

To confirm the creation of the interface you can run (first `source install/setup.zsh`):

```shell
ros2 interface show tutorial_interfaces/msg/Num
```

the output should be `int64 num`.

### Test the new interface

`cd` into the `cpp_pubsub` package.  Then add to `package.xml`

```xml
<depend>tutorial_interfaces</depend>
```

We need only to modify the lines related to the messages of both the publisher and the subscriber.

Firs the publisher:

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp" // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
	MinimalPublisher()
		: Node("minimal_publisher"), count_(0)
	{
		// CHANGE
		publisher_ =  this->create_publisher 
		<tutorial_interfaces::msg::Num>("topic", 10); 

		auto timer_callback = [this](){
		    auto message = 
			    tutorial_interfaces::msg::Num(); // CHANGE
				
	    message.num = this->count_++; // CHANGE
		
	    RCLCPP_INFO_STREAM(
		    this->get_logger(), 
		    "Publishing: '" << message.num << "'"
	    ); // CHANGE
		
      publisher_->publish(message);
    };
	
	    timer_ = this->create_wall_timer(
		    500ms,  timer_callback
		);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	// CHANGE
	rclcpp::Publisher 
	<tutorial_interfaces::msg::Num>::SharedPtr  publisher_; 
	size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

As we can see we access the custom message with `tutorial_interfaces::msg::Num`:

- `tutorial_interfaces`: first namespace, the name is given by the package name.
- `msg`: second namespace (grouping message definitions).
- `Num`: name of the class (also name of the file `Num.msg` that we have created).

Similarly modify the subscriber:

```cpp
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp" // CHANGE

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        
        auto topic_callback = 
            [this](
				// CHANGE
	            const tutorial_interfaces::msg::Num & msg
	        ) { 
				RCLCPP_INFO_STREAM(
					this->get_logger(), 
					"I heard: '" 
					<< msg.num << "'"
				); // CHANGE
		    };

        subscription_ =
            this->create_subscription
            <tutorial_interfaces::msg::Num>(
                "topic", 10, topic_callback
            );
    }

private:
	// CHANGE
    rclcpp::Subscription
    <tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

Modify `CMakeList.txt` with:

```cmake
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED) # CHANGE

add_executable(talker src/publisher_member_function.cpp)

# CHANGE
ament_target_dependencies(talker rclcpp tutorial_interfaces)   

add_executable(listener src/subscriber_member_function.cpp)

# CHANGE
ament_target_dependencies(listener rclcpp tutorial_interfaces) 

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

Finally compile with:

 ```shell
 colcon build --packages-select cpp_pubsub
 ```

Another example based on publisher/subscriber can be found at [[Homework 1]].