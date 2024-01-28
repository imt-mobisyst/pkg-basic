# Let's play with ROS2...

Documentation on docs.ros.org:

- [Installation](https://docs.ros.org/en/humble/Installation.html).
- [Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

## Install:

```sh
sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install ros-humble-desktop
```

You can check the environement variables: 

```sh
source /opt/ros/humble/setup.bash # .zsh
printenv | grep -i ROS_
```

(optional) Finally you can add a `domain-id` to allow several ROS2 virtual networks to work on a same phisical network ([doc](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html))

```
export ROS_DOMAIN_ID=42
```

Push the `source` and `export` commands on your bashrc file.

## Hello World: Talker-listener

In 2 shells: 

```sh
source /opt/ros/humble/setup.bash # .zsh
ros2 run demo_nodes_cpp talker
```

and:

```sh
source /opt/ros/humble/setup.bash # .zsh
ros2 run demo_nodes_py listener
```

## Some usefull ros2 command:

command      | description
-------------|----------------
`action`     | Various action related sub-commands
`bag`        | Various rosbag related sub-commands
`component`  | Various component related sub-commands
`daemon`     | Various daemon related sub-commands
`doctor`     | Check ROS setup and other potential issues
`interface`  | Show information about ROS interfaces
`launch`     | Run a launch file
`lifecycle`  | Various lifecycle related sub-commands
`multicast`  | Various multicast related sub-commands
`node`       | Various node related sub-commands
`param`      | Various param related sub-commands
`pkg`        | Various package related sub-commands
`run`        | Run a package specific executable
`security`   | Various security related sub-commands
`service`    | Various service related sub-commands
`topic`      | Various topic related sub-commands
`wtf`        | Use `wtf` as alias to `doctor`

Call `ros2 <command> -h` for more detailed usage.

## Python Talker-listener:

Talker:

```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```


Listener: 

 ```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

# Play with TurtleSim

```sh
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
