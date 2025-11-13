# Services 

> Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![[Pasted image 20251113200134.png]]

When nodes communicate using services, the node that sends a request for data is called the **client node**, and the one that responds to the request is the **service node**. The structure of the request and response is determined by a  `.srv` file.

In the example below we consider a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

Before starting create the package with:

```shell
ros2 pkg create \
--build-type ament_cmake \
--license Apache-2.0 cpp_srvcli \
--dependencies rclcpp example_interfaces
```

### Server node

File: `add_two_ints_server.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

// Function used to add the two integers.
void add(
	const
    std::shared_ptr
    <example_interfaces::srv::AddTwoInts::Response>
    response
) {

    // Compute the sum.
    response->sum = request->a + request->b;
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"), 
        "Incoming request\na: %ld" " b: %ld",
        request->a, request->b
    );
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "sending back response: [%ld]",
        (long int)response->sum
    );
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    // Create a new node.
    std::shared_ptr<rclcpp::Node> node = 
        rclcpp::Node::make_shared("add_two_ints_server");

    // Create a service for the node, advertise it over 
    // the networks with the &add method.
    rclcpp::Service
    <example_interfaces::srv::AddTwoInts>::SharedPtr 
    service = 
	    node->create_service
		    <example_interfaces::srv::AddTwoInts>
		    ("add_two_ints", &add);

    RCLCPP_INFO(
	    rclcpp::get_logger("rclcpp"), 
	    "Ready to add two ints."
	);

    rclcpp::spin(node);
    rclcpp::shutdown();
}
```

### Client node

File: `add_two_ints_client.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "usage: add_two_ints_client X Y"
        );
        return 1;
    }

    // Create a new node.
    std::shared_ptr<rclcpp::Node> node = 
        rclcpp::Node::make_shared("add_two_ints_client");
    
    // Crate a client for the node.
    rclcpp::Client
    <example_interfaces::srv::AddTwoInts>::SharedPtr 
    client =
        node->create_client
        <example_interfaces::srv::AddTwoInts>
        ("add_two_ints");

    // Create the request.
    auto request = 
        std::make_shared
        <example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    // This gives the client 1 second to search 
    // for service nodes in the network.
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"), 
                "Interrupted while waiting for "
                "the service. Exiting."
            );
            return 0;
        }
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), 
            "service not available, waiting again..."
        );
    }

    // Send the request to the server.
    auto result = client->async_send_request(request);

    // Wait for the result.
    if (
        rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS
    ) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "Sum: %ld",
            result.get()->sum
        );
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Failed to call service add_two_ints"
        );
    }

    rclcpp::shutdown();
    return 0;
}
```

Finally the `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```