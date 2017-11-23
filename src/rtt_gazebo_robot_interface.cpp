#include "rtt_gazebo_robot_interface/rtt_gazebo_robot_interface.h"
using namespace RTT;

RttGazeboRobotInterface::RttGazeboRobotInterface(const std::string& name)
: TaskContext(name)
{
    this->provides("command")->addPort("JointTorque",port_jnt_trq_in_);
    this->provides("state")->addPort("JointTorque",port_jnt_trq_in_);
    this->provides("state")->addPort("JointPosition",port_jnt_trq_in_);
    this->provides("state")->addPort("JointVelocity",port_jnt_trq_in_);
    
}

bool RttGazeboRobotInterface::configureHook()
{
    log(Info) << "[" << getName() << "] " << " Getting gazebo world" << endlog();
    gazebo::printVersion();
    auto world = gazebo::physics::get_world();
    if(! world)
    {
        log(Error) << "[" << getName() << "] " << "Could not load gazebo world. Make sure the server is launched (rtt_gazebo_embedded)" << endlog();
        return false;
    }
    
    log(Info) << "[" << getName() << "] " << " World loaded, getting model " << getName() << endlog();
    gazebo_model_ = world->ModelByName( getName() );
    
    if(! gazebo_model_)
    {
        log(Error) << "[" << getName() << "] " << "Could not get gazebo model " << getName() << ". Make sure it is loaded in the server" << endlog();
        return false;
    }
    
    log(Info) << "[" << getName() << "] " << " Getting actuated joint names" << endlog();
    joint_map_.clear();
    for(auto joint : gazebo_model_->GetJoints() )
    {
        // Not adding fixed joints
        if(joint->GetType() != gazebo::physics::Joint::FIXED_JOINT 
            && joint->GetLowerLimit(0u) != joint->GetUpperLimit(0u)
            && joint->GetLowerLimit(0u) != 0)
        {
            log(Info) << "[" << getName() << "] " << " Adding joint " << joint->GetName() << endlog();
            joint_map_.push_back(joint->GetName());
        }
        else
        {
            log(Info) << "[" << getName() << "] " << " Not adding joint " << joint->GetName() << endlog();
        }
    }
    
    if(joint_map_.size() == 0)
    {
        log(Error) << "[" << getName() << "] " << "Could not get any actuated joint for model " << getName() << endlog();
        return false;
    }
    
    for(auto joint_name : joint_map_)
    {
        std::cout << "   - " << joint_name << std::endl;
    }
    
    log(Info) << "[" << getName() << "] " << " Connecting Gazebo events" << endlog();
    gazebo_world_begin_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RttGazeboRobotInterface::worldUpdateBegin,this));
    gazebo_world_end_connection_   = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RttGazeboRobotInterface::worldUpdateEnd,this));
    
    return true;
}

void RttGazeboRobotInterface::worldUpdateBegin()
{
    
}

void RttGazeboRobotInterface::worldUpdateEnd()
{
    
}

void RttGazeboRobotInterface::updateHook()
{

}

ORO_CREATE_COMPONENT(RttGazeboRobotInterface)