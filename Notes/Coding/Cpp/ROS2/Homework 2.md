# Exercise: custom action

Similarly to homework 1, we have a roomba like robot. In this scenario we want to charge the robot using an action. 

| **Component**            | **Description**                                                                             |
| ------------------------ | ------------------------------------------------------------------------------------------- |
| **Scenario**             | Robot is already attached to the charging station with low battery (e.g., 5%)               |
| **Communication Method** | ROS Action                                                                                  |
| **Action Client**        | Charging Station - sends goal request with charging power ($p \leq 100$)                    |
| **Action Server**        | Robot - can accept or reject the charging request                                           |
| **Charging Process**     | If accepted, CS starts charging procedure                                                   |
| **Charging Duration**    | 1 minute to reach $min(100, b + p)$ where $b$ is current battery level                      |
| **Feedback**             | Robot continuously publishes current battery level to action feedback topic during charging |
| **Completion**           | Robot sends result to CS when battery reaches $min(100, b+p)$.                              |

In the provided solution we assume that the goal is always accepted by the server. The starting battery of the robot is a random number between 0 and 8. The power supplied by the charger is a random number between 80 and 100.

## Solution

File `vacuum_action_interfaces/action/Charging.action`:

```
uint32 power
---
uint32 new_battery
---
uint32 curr_battery
```

File `vacuum_charger/src/vacuum_server.cpp`:

```cpp
#include <functional>
#include <memory>
#include <thread>
#include <algorithm>

#include "vacuum_charger/visibility_control.h"
#include "vacuum_action_interfaces/action/charging.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vacuum_charger {

class VacuumActionServer: public rclcpp::Node {
public:

    using Charging = vacuum_action_interfaces::action::Charging;
    using GoalHandleCharging = rclcpp_action::ServerGoalHandle<Charging>;

    CUSTOM_ACTION_CPP_PUBLIC
    explicit VacuumActionServer(
		const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    ) : Node( "vacuum_action_server", options) {
        using namespace std::placeholders;

        // Just accept all new goals.
        auto handle_goal = 
            [this](
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Charging::Goal> goal
            ) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received goal: power supplied %u",
                    goal->power
                );
                (void)uuid;  // prevent -Wunused warnings.
                return rclcpp_action::
                GoalResponse::ACCEPT_AND_EXECUTE;
            };

        // Just accept all cancellation.
        auto handle_cancel = 
            [this](
	            const std::shared_ptr<GoalHandleCharging> 
	            goal_handle
            ) {
                RCLCPP_INFO(
	                this->get_logger(),
	                "Received request to cancel goal"
		        );
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        // Start processing the goal.
        auto handle_accepted = 
            [this](
	            const std::shared_ptr<GoalHandleCharging> goal_handle
	        ) {
                auto execute_in_thread = 
                    [this, goal_handle]() {
                        // See 'execute' function below.
                        return this->execute(goal_handle);
                    };
                std::thread{execute_in_thread}.detach();
            };

        // Define an action server.
        this->action_server_ = 
	    rclcpp_action::create_server<Charging>(
            this,
            "charging",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }

private:
    rclcpp_action::Server<Charging>::SharedPtr action_server_;

    // Charging operation.
    void execute(
	    const std::shared_ptr<GoalHandleCharging> goal_handle
    ) {
        // Define goal.
        const auto goal = goal_handle->get_goal();
        const uint power = goal->power;

        // Define feedback.
        auto feedback = std::make_shared<Charging::Feedback>();
        // Init current battery: random between 0-8.
        feedback->curr_battery = rand() % 8;

        // Define the result object.
        auto result = std::make_shared<Charging::Result>();

        // Compute the next element in Charging sequence.
        const uint final_power = 
            std::min(100u, power + feedback->curr_battery);

        // One itration per second.
        rclcpp::Rate loop_rate(final_power / 60.);

        // Define a function used to publish feedback.
        auto publish_callback_lambda =
            [this, goal_handle, feedback]() {
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(),"Publish feedback");
            };

        rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
            std::chrono::seconds(1), // Update feedback every second.
            publish_callback_lambda
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Executing goal...\n\tstarting level: %u, final level: %u",
            feedback->curr_battery, final_power
        );

        // Charging loop.
        const int itr = final_power - feedback->curr_battery;
        for (int i = 0; (i < itr)&& rclcpp::ok(); ++i) {

            // Check if there is a cancel request.
            if (goal_handle->is_canceling()) {
                result->new_battery = feedback->curr_battery;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Increase battery.
            feedback->curr_battery += 1;

            loop_rate.sleep(); // Wait.
        }

        // Check if goal is done.
        if (rclcpp::ok()) {
            // Save result.
            result->new_battery = feedback->curr_battery;
            goal_handle->succeed(result);
            RCLCPP_INFO(
	            this->get_logger(), 
	            "Goal succeeded"
            );
        }
    };

};  // class VacuumActionServer

}  // namespace vacuum_charger

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_charger::VacuumActionServer)
```

File `vacuum_charger/src/charger_client.cpp`:

```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "vacuum_action_interfaces/action/charging.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vacuum_charger {

class ChargerActionClient: public rclcpp::Node {
public:
    using Charging = vacuum_action_interfaces::action::Charging;
    using GoalHandleCharging = rclcpp_action::ClientGoalHandle<Charging>;

    explicit ChargerActionClient(const rclcpp::NodeOptions& options)
    : Node("charger_action_client", options) {
        // Define an action client.
        this->client_ptr_ = 
            rclcpp_action::create_client<Charging>(
                this,
                "charging"
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

        // Define a (Charging) goal message.
        auto goal_msg = Charging::Goal();
        goal_msg.power = (uint) (60 + rand() % 40);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // Define Response, Feedback and Result callbacks.
        auto send_goal_options = 
            rclcpp_action::Client<Charging>::SendGoalOptions();
        
        // ### Response callback ###.
        send_goal_options.goal_response_callback = 
            [this](const GoalHandleCharging::SharedPtr& goal_handle) {
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
        send_goal_options.feedback_callback = 
            [this](
                GoalHandleCharging::SharedPtr,
                const std::shared_ptr<const Charging::Feedback> feedback
            ) {
                // Log current battery of vacuum.
                RCLCPP_INFO(
	                this->get_logger(),
                    "Current battery level: %u", feedback->curr_battery
	            );
            };

        // ### Result callback ###.
        send_goal_options.result_callback = 
            [this](
	            const GoalHandleCharging::WrappedResult& result
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
                RCLCPP_INFO(
	                this->get_logger(),
                    "Final battery level: %u", result.result->new_battery
                );
                rclcpp::shutdown();
            };

        // Send goal to the server.
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
  rclcpp_action::Client<Charging>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // class ChargingActionClient

}  // namespace vacuum_charger

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_charger::ChargerActionClient)
```