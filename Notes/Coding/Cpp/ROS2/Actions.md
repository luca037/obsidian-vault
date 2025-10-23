# Understanding actions

> Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

TODO: put the image.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model. An `action client` node sends a goal to an `action server` node that acknowledges the goal and returns a stream of feedback and a result.

# Creating a custom action

First we need to create an interface packge.

```shell
ros2 pkg create --license Apache-2.0 custom_action_interfaces
```

Actions are defined in `.action` files with the form:

```
# Request
---
# Result
---
# Feedback
```

where:

1. A **request message** is sent from an action client to an action server initiating a new goal.
2. A **result message** is sent from an action server to an action client when a goal is done.
3. **Feedback messages** are periodically sent from an action server to an action client with updates about a goal.

An instance of an action is typically referred to as a *goal*.

In this tutorial we create an action "Fibonacci" for computing the Fibonacci sequence. Our custom action is defined inside the package that we have just created. We create the `action` directory and inside we create `Fibonacci.action`:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Then we add the following lines to `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

We add the dependencies to `package.xml`:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Finally we can compile the package.

After sourcing our workspace, we can check our action definition exists:

```shell
ros2 interface show custom_action_interfaces/action/Fibonacci
```

# Writing an action server and client

First we create the package:

```shell
ros2 pkg create \
--dependencies custom_action_interfaces rclcpp \
rclcpp_action  rclcpp_components \
--license Apache-2.0 \
-- custom_action_cpp
```

**Note**: we have added all the necessary dependencies with `--denepdencies` so we don't have to manually edit `package.xml`
and `CMakeLists.txt`.

To make the package compile and work in windows we need to create the file `custom_action_cpp/include/custom_action_cpp/visibility_control.h`. Write in it the following lines:

```cpp
#ifndef CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_
#define CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CUSTOM_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define CUSTOM_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CUSTOM_ACTION_CPP_EXPORT __declspec(dllexport)
    #define CUSTOM_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
    #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
  #else
    #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
  #endif
  #define CUSTOM_ACTION_CPP_PUBLIC_TYPE CUSTOM_ACTION_CPP_PUBLIC
  #define CUSTOM_ACTION_CPP_LOCAL
#else
  #define CUSTOM_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CUSTOM_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CUSTOM_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CUSTOM_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CUSTOM_ACTION_CPP_PUBLIC
    #define CUSTOM_ACTION_CPP_LOCAL
  #endif
  #define CUSTOM_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_
```

### Action Server

Create `src/src/fibonacci_action_server.cpp`:

```cpp
#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"

namespace custom_action_cpp {

class FibonacciActionServer : public rclcpp::Node {
public:

    using Fibonacci = 
    custom_action_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = 
    rclcpp_action::ServerGoalHandle<Fibonacci>;

    CUSTOM_ACTION_CPP_PUBLIC

    explicit FibonacciActionServer(
		const rclcpp::NodeOptions& options
		=rclcpp::NodeOptions()
    )
    : Node(
	    "fibonacci_action_server", // Name of the node.
	    options
	) {
        using namespace std::placeholders;

        // Function to handle the goal.
        // Just accept all new goals.
        auto handle_goal = 
            [this](
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Fibonacci::Goal> goal
            ) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received goal request with order %d",
                    goal->order
                );
                (void)uuid;  // prevent -Wunused warnings.
                return rclcpp_action::
                GoalResponse::ACCEPT_AND_EXECUTE;
            };

        // Function to cancel the goal.
        // Just accept all cancellation.
        auto handle_cancel = 
            [this](
	            const std::shared_ptr<GoalHandleFibonacci> 
	            goal_handle
            ) {
                RCLCPP_INFO(
	                this->get_logger(),
	                "Received request to cancel goal"
		        );
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        // Function to handle accepted goals.
        // Start processing the goal.
        auto handle_accepted = 
            [this](
	            const std::shared_ptr<GoalHandleFibonacci> 
	            goal_handle
	        ) {
                // This needs to return quickly to avoid 
                // blocking the executor, 
                // so we declare a lambda function 
                // to be called inside a new thread.
                auto execute_in_thread = 
                    [this, goal_handle]() {
                        // See 'execute' function below.
                        return this->execute(goal_handle);
                    };
                // Spaw a thread to do the 
                // actual work and return...
                std::thread{execute_in_thread}.detach();
            };

        // Define a new action server.
        // It requrires 6 things:
        //  1. The templated action type name: Fibonacci.
        //  2. A ROS 2 node to add the action to: this.
        //  3. The action name: 'fibonacci'.
        //  4. A callback function for handling goals: 
		//     handle_goal
        //  5. A callback function for handling 
		//     cancellation: handle_cancel.
        //  6. A callback function for handling 
        //     goal accept: handle_accept.
        this->action_server_ = 
	    rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }

private:
    // The action server
    rclcpp_action::Server<Fibonacci>::
    SharedPtr action_server_;

    // All further processing and updates are done in 
    // this function, that is running on the new thread.
    void execute(
	    const std::shared_ptr<GoalHandleFibonacci>
	    goal_handle
    ) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        // One itration per second.
        rclcpp::Rate loop_rate(1);

        // Get the goal: needed to retreive the 
        // 'order' of Fibonacci.
        const auto goal = goal_handle->get_goal();

        // Define the feedback object and get 
        //the partial sequence list.
        auto feedback = 
	        std::make_shared<Fibonacci::Feedback>();
        auto& sequence = feedback->partial_sequence;

        // Push back first two numbers.
        sequence.push_back(0);
        sequence.push_back(1);

        // Define the result object.
        auto result = std::make_shared<Fibonacci::Result>();

        // Compute the next element in Fibonacci sequence.
        for (
	        int i = 1; (i < goal->order) && rclcpp::ok(); ++i
        ) {

            // Check if there is a cancel request.
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(
	                this->get_logger(),
	                "Goal canceled"
                );
                return;
            }

            // Update sequence.
            sequence.push_back(
	            sequence[i] + sequence[i - 1]
            );

            // Publish feedback.
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(
	            this->get_logger(),
	            "Publish feedback"
            );

            loop_rate.sleep(); // Wait for 1 second.
        }

        // Check if goal is done.
        if (rclcpp::ok()) {
            // Save result.
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(
	            this->get_logger(), 
	            "Goal succeeded"
            );
        }
    };

};  // class FibonacciActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionServer)
```

Add to `CMakeLists.txt`:

```cmake
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "custom_action_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## Action Client

Create file `src/fibonacci_action_client.cpp`:

```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "custom_action_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom_action_cpp {

class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = 
    custom_action_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = 
    rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(
	    const rclcpp::NodeOptions& options
	)
    : Node(
	    "fibonacci_action_client", // Name of the node.
	    options
    ) {
        // Define a new action client.
        // An action client requires 3 things:
        //  1. The templated action type name: Fibonacci.
        //  2. A ROS 2 node to add the action 
        //     client to: this.
        //  3. The action name: 'fibonacci'.
        this->client_ptr_ = 
            rclcpp_action::create_client<Fibonacci>(
                this,
                "fibonacci"
            );

        // Instantiate a ROS timer that will kick off the 
        // one and only call to send_goal function.
        auto timer_callback_lambda = 
	        [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            timer_callback_lambda
        );
    }

    void send_goal() {
        using namespace std::placeholders;

        // Cancels the timer (so it is only called once).
        this->timer_->cancel();

        // Waits for the action server to come up.
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available after waiting"
            );
            rclcpp::shutdown();
        }

        // Define a (Fibonacci) goal message.
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // Now we set the Response, 
        // Feedback and Result callbacks.
        auto send_goal_options = 
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        
        // ### Response callback ###.
        // When the server receives and accepts the goal, 
        // it will send a response to the client.
        // That response is handled by this function.
        send_goal_options.goal_response_callback = 
            [this](
	            const GoalHandleFibonacci::
	            SharedPtr& goal_handle
            ) {
                if (!goal_handle)
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was rejected by server"
	                );
                else
                    RCLCPP_INFO(
	                    this->get_logger(), 
	                    "Goal accepted by server,"
	                    "waiting for result"
	                );
            };

        // ### Feedback callback ###.
        // Assuming the goal was accepted by the 
        // server, it will start processing. 
        // Any feedback to the client will be 
        // handled by this function.
        send_goal_options.feedback_callback = 
            [this](
                GoalHandleFibonacci::SharedPtr,
                const std::shared_ptr
                <const Fibonacci::Feedback> feedback
            ) {
                // Log the received partial sequence.
                std::stringstream ss;
                ss << "Next number in sequence received: ";
                for (auto number:feedback->partial_sequence)
                    ss << number << " ";
                RCLCPP_INFO(
	                this->get_logger(),
	                ss.str().c_str()
	            );
            };

        // ### Result callback ###.
        // When the server is finished processing, it will 
        // return a result to the client. 
        // The result is handled by the following function.
        send_goal_options.result_callback = 
            [this](
	            const GoalHandleFibonacci::
	            WrappedResult& result
	        ) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was aborted"
	                );
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was canceled"
	                );
                    return;
                default:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Unknown result code"
	                );
                    return;
                }
                // Log result received.
                std::stringstream ss;
                ss << "Result received: ";
                for (auto number : result.result->sequence)
                    ss << number << " ";
                RCLCPP_INFO(
	                this->get_logger(),
	                ss.str().c_str()
                );
                rclcpp::shutdown();
            };
        // Send goal to the server.
        this->client_ptr_->
        async_send_goal(goal_msg, send_goal_options);
    }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // class FibonacciActionClient

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionClient)

```

Add to `CMakeLists.txt`:

```cmake
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "custom_action_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## Build and run

After sourcing workspace.

```shell
colcon build
ros2 run custom_action_cpp fibonacci_action_server
ros2 run custom_action_cpp fibonacci_action_client
```
