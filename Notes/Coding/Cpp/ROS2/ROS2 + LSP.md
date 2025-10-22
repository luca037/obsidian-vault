# LSP with clangd

Source tutorial -> [link](https://chenbrian.ca/posts/ros2_lsp_setup/)

Add default mixins files:

```shell
colcon mixin add default \
"https://raw.githubusercontent.com/colcon/colcon-mixin-
repository/master/index.yaml"
```

Fetch them:

```shell
colcon mixin update default
```

Use them:

```shell
colcon build --mixin compile-commands
```