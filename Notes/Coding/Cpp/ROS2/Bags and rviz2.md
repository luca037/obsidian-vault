# Bags

> `ros2 bag` is a command line tool for recording data published on topics and services in your ROS 2 system. It accumulates the data passed on any number of topics and services, then saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics and services is also a great way to share your work and allow others to recreate it.

To **record the data** published to a topic use the command syntax:

```shell
ros2 bag record <topic_name>
```

You can **see details** about your recording by running:

```shell
ros2 bag info <bag_file_name>
```

To **play topic data**: 

```shell
ros2 bag play <bag_file_name>
```

### Example: \tf topic

In our case we have a directory called `es_bag2`, inside we have two files: `es_bag2.mcap`, `metadata.yaml`.

To run our bag, first create two terminal. In one run the following command:

```shell
ros2 bag play es_bag2.mcap
```

In the other one run:

```shell
ros2 topic echo tf
```

Here `tf` stands for `transformations` and is the pre-defined topic in which the data of our bag will be published.

In the output stream we should see something like:

```
transforms:
- header:
    stamp:
      sec: 1756890218
      nanosec: 294356445
    frame_id: camera_color_optical_frame
  child_frame_id: tag36h11:0
  transform:
    translation:
      x: 0.13433121217759
      y: -1.5453146851452075
      z: 3.300292537854008
    rotation:
      x: -0.05226988186230123
      y: 0.8646427518948953
      z: 0.4992706498042989
      w: 0.01973801635718969
```

In this case we can see some important information about the data that we're receiving:

1. We have one reference frame called `camera_color_optical` which is our `frame_id`. This is the **parent frame**.
2. The other reference frame is `tag36h11:0`, this is our `child_frame_id`.
3. The transform sections tells us the transformation from the **parent frame** to the **child frame**. So in this case we can see that the child frame is translated w.r.t. the parent frame, and it is also rotated.

Those information will be useful to better understand [[Homework 3]].

# rviz2

> RViz2 is a 3D visualization tool for the Robot Operating System (ROS) 2, allowing users to visualize robot models, sensor data, and maps in a graphical interface.

We will not cover all the aspect of `rviz2`, here we see how to visualize the data that are published in the pre-created bag described in the previous example.

You can find a proper introduction to `rviz2` at this [link](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).

### Visualizing bag data

The goal is to visualize the data that our bag is publishing in `\tf` topic.

To do so, first we need to create two terminal. In one of the two type `rviz2` to open the graphical software. In the other one run the following command:

```shell
ros2 bag play es_bag2.mcap --loop
```

This will run the bag in *loop*, so we don't have to re-run again the command every time.

Now switch to `rviz2` window. At first you will see nothing in the 3D plane. We need to:

1. Add a new display: **Add -> TF**. The add button is on the bottom left corner.
2. In the left context menu, find **Global Options** section and change **fixed frame** value to **tag36h11:0**.
3. Find **TF** section on the left context menu, check the box **Show Names** and deselect all the **Frames** except for the three frames *tag36h11:1,2,3* and *camera_link*.

If you don't see anything moving, then click on the **Reset** button on the bottom left corner to reset ros2 timer.

With this setup you're ready for [[Homework 3]].