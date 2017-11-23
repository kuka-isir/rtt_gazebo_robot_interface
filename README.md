RTT Gazebo Default Robot Interface
==================================

This package provides a default robot interface to `rtt_gazebo_embedded`.

#### Ports


```ruby
# Input
robot.command.JointTorque
# Outputs
robot.state.JointTorque
robot.state.JointPosition
robot.state.JointVelocity
```

#### Usage

First you need to load Gazebo inside the orocos deployer with `rtt_gazebo_embedded`.

In a terminal : `deployer -l info`


```ruby

var string urdf = "/path/to/myrobot.urdf"

# Gazebo 
import("rtt_gazebo_embedded")
loadComponent("gazebo","RTTGazeboEmbedded")
setActivity("gazebo",0,10,ORO_SCHED_OTHER)
gazebo.world_path = "worlds/empty.world"
gazebo.configure()
gazebo.insertModelFromURDF( urdf )
gazebo.start()

```

Then you can load the interfacefor your model. The default behavior is to set the `model_name` to interface to the name of the orocos component.

```ruby
# Default Gazebo model interface
import("rtt_gazebo_robot_interface")
loadComponent("lwr","RttGazeboRobotInterface")
# The component is aperiodic and is triggered with gazebo
setActivity("lwr",0,10,ORO_SCHED_RT)

# Loads the model. The default name is the name of the orocos Component
lwr.configure()
# Setting its initial configuration
lwr.setModelConfiguration(strings("joint_0","joint_1","joint_3","joint_5"),array(-2.2,-0.1,1.47,-1.47))

lwr.start()
```

You can also override the default name (before `configure()`) :

```ruby
# Default Gazebo model interface
import("rtt_gazebo_robot_interface")
loadComponent("robot","RttGazeboRobotInterface")
setActivity("robot",0,10,ORO_SCHED_RT)
# Set the model name. This has to be called before configure()
robot.model_name = "lwr"
robot.configure()
# Setting its initial configuration
robot.setModelConfiguration(strings("joint_0","joint_1","joint_3","joint_5"),array(-2.2,-0.1,1.47,-1.47))

robot.start()
```
