#include "rtt_gazebo_robot_interface/rtt_gazebo_robot_interface.h"
#include <cmath>
using namespace RTT;

RttGazeboRobotInterface::RttGazeboRobotInterface(const std::string& name)
: TaskContext(name)
, model_name_(name)
{
    this->provides("command")->addPort("JointTorque",port_jnt_trq_in_);
    this->provides("state")->addPort("JointTorque",port_jnt_trq_out_);
    this->provides("state")->addPort("JointPosition",port_jnt_pos_out_);
    this->provides("state")->addPort("JointVelocity",port_jnt_vel_out_);
    this->provides("state")->addPort("WorldToBase",port_world_to_base_out_);
    this->provides("state")->addPort("BaseVelocity",port_base_vel_out_);
    this->provides("state")->addPort("Gravity",port_gravity_out_);

    this->provides("state")->addOperation("print",&RttGazeboRobotInterface::printState,this,RTT::OwnThread);
    this->addProperty("model_name",model_name_).doc("The name of the model to load");
    this->addOperation("setModelConfiguration",&RttGazeboRobotInterface::setModelConfiguration,this,RTT::OwnThread);

    this->provides("world")->addOperation("getGravity",&RttGazeboRobotInterface::getGravity,this,RTT::OwnThread);
    this->addOperation("getNrOfDegreesOfFreedom",&RttGazeboRobotInterface::getNrOfDegreesOfFreedom,this,RTT::OwnThread);
}

int RttGazeboRobotInterface::getNrOfDegreesOfFreedom()
{
    return joint_map_.size();
}

void RttGazeboRobotInterface::printState()
{
    std::cout << "Model " << getName() << '\n';
    std::cout << "JointPosition : " << '\n';
    std::cout << current_jnt_pos_.transpose() << '\n';
    std::cout << "JointVelocity : " << '\n';
    std::cout << current_jnt_vel_.transpose() << '\n';
    std::cout << "JointTorque : " << '\n';
    std::cout << current_jnt_trq_.transpose() << '\n';
    std::cout << "Physical joints" << std::endl;
    for(auto joint_name : joint_map_)
    {
        std::cout << "   - " << joint_name << std::endl;
    }
}

Eigen::Vector3d RttGazeboRobotInterface::getGravity()
{
    return gravity_;
}

bool RttGazeboRobotInterface::setModelConfiguration(std::vector<std::string> joint_names,std::vector<double> joint_positions)
{
    if (!gazebo_model_)
    {
        log(Error) << "[" << getName() << "] " << "Model is not loaded" << endlog();
        return false;
    }

    if (joint_names.size() != joint_positions.size())
    {
        printState();
        log(Error) << "[" << getName() << "] " << "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << endlog();
        return false;
    }

    auto world = gazebo::physics::get_world();
    // make the service call to pause gazebo
    bool is_paused = world->IsPaused();
    if (!is_paused) world->SetPaused(true);

    std::map<std::string, double> joint_position_map;
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      joint_position_map[joint_names[i]] = joint_positions[i];
    }

    gazebo_model_->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world->SetPaused(is_paused);

    return true;
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

    log(Info) << "[" << getName() << "] " << " World loaded, getting model " << model_name_ << endlog();
    gazebo_model_ = world->GetModel( model_name_ );

    if(! gazebo_model_)
    {
        log(Error) << "[" << getName() << "] " << "Could not get gazebo model " << model_name_ << ". Make sure it is loaded in the server" << endlog();
        return false;
    }

    log(Info) << "[" << getName() << "] " << " Getting actuated joint names" << endlog();
    joint_map_.clear();
    for(auto joint : gazebo_model_->GetJoints() )
    {
        // Not adding fixed joints
        if( true
#if GAZEBO_VERSION_MAJOR >= 7
            && joint->GetType() != gazebo::physics::Joint::FIXED_JOINT
#endif
            && joint->GetLowerLimit(0u) != joint->GetUpperLimit(0u)
            && joint->GetLowerLimit(0u).Radian() != 0
            && !std::isnan(joint->GetLowerLimit(0u).Radian())
            && !std::isnan(joint->GetUpperLimit(0u).Radian()))
        {
            log(Info) << "[" << getName() << "] " << " Adding joint " << joint->GetName()
                      << " type " << joint->GetType()
                      << " lower limit " << joint->GetLowerLimit(0u)
                      << " upper limit " << joint->GetUpperLimit(0u)
                      << endlog();
            joint_map_.push_back(joint->GetName());
        }
        else
        {
            log(Info) << "[" << getName() << "] " << " Not adding joint " << joint->GetName()
                      << " type " << joint->GetType()
                      << " lower limit " << joint->GetLowerLimit(0u)
                      << " upper limit " << joint->GetUpperLimit(0u)
                      << endlog();        }
    }

    if(joint_map_.size() == 0)
    {
        log(Error) << "[" << getName() << "] " << "Could not get any actuated joint for model " << model_name_ << endlog();
        return false;
    }

    std::cout << "Physical joints" << std::endl;
    for(auto joint_name : joint_map_)
    {
        std::cout << "   - " << joint_name << std::endl;
    }

    current_jnt_pos_.setZero(joint_map_.size());
    current_jnt_vel_.setZero(joint_map_.size());
    current_jnt_trq_.setZero(joint_map_.size());
    jnt_trq_command_.setZero(joint_map_.size());

    log(Info) << "[" << getName() << "] " << " Connecting Gazebo events" << endlog();
    gazebo_world_begin_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RttGazeboRobotInterface::worldUpdateBegin,this));
    gazebo_world_end_connection_   = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RttGazeboRobotInterface::worldUpdateEnd,this));

    return true;
}

void RttGazeboRobotInterface::worldUpdateBegin()
{
    if(!isRunning())
    {
        return;
    }

    if (!gazebo_model_)
    {
        log(Error) << "[" << getName() << "] " << "Model is not loaded" << endlog();
        return;
    }

    if(port_jnt_trq_in_.readNewest(jnt_trq_command_) != RTT::NoData)
    {
        gazebo_model_->SetEnabled(true);

        for(int i=0 ; i < joint_map_.size() ; ++i)
        {
            auto joint = gazebo_model_->GetJoint( joint_map_[i] );
            joint->SetForce(0,jnt_trq_command_[i]);
        }
    }
    else
    {
        gazebo_model_->SetEnabled(false);
        for(auto joint : gazebo_model_->GetJoints())
            joint->SetVelocity(0, 0.0);
    }
}

void RttGazeboRobotInterface::worldUpdateEnd()
{
    if(!isRunning())
    {
        return;
    }

    if (!gazebo_model_)
    {
        log(Error) << "[" << getName() << "] " << "Model is not loaded" << endlog();
        return;
    }

    auto world = gazebo::physics::get_world();

    for(int i=0 ; i < joint_map_.size() ; ++i)
    {
        auto joint = gazebo_model_->GetJoint( joint_map_[i] );
        current_jnt_pos_[i] = joint->GetAngle(0).Radian();
        current_jnt_vel_[i] = joint->GetVelocity(0);
        current_jnt_trq_[i] = joint->GetForce(0u);
    }


    auto pose = gazebo_model_->GetRelativePose();
    current_world_to_base_.translation() = Eigen::Vector3d(pose.pos.x,pose.pos.y,pose.pos.z);
    current_world_to_base_.linear() = Eigen::Quaterniond(pose.rot.w,pose.rot.x,pose.rot.y,pose.rot.z).toRotationMatrix();

    auto base_vel_lin = gazebo_model_->GetRelativeLinearVel();
    auto base_vel_ang = gazebo_model_->GetRelativeAngularVel();
    current_base_vel_[0] = base_vel_lin.x;
    current_base_vel_[1] = base_vel_lin.y;
    current_base_vel_[2] = base_vel_lin.z;
    current_base_vel_[3] = base_vel_ang.x;
    current_base_vel_[4] = base_vel_ang.y;
    current_base_vel_[5] = base_vel_ang.z;

    auto grav = world->GetPhysicsEngine()->GetGravity();

    gravity_[0] = grav[0];
    gravity_[1] = grav[1];
    gravity_[2] = grav[2];

    port_gravity_out_.write(gravity_);
    port_base_vel_out_.write(current_base_vel_);
    port_world_to_base_out_.write(current_world_to_base_.matrix());
    port_jnt_pos_out_.write(current_jnt_pos_);
    port_jnt_vel_out_.write(current_jnt_vel_);
    port_jnt_trq_out_.write(current_jnt_trq_);
}

void RttGazeboRobotInterface::updateHook()
{

}

ORO_CREATE_COMPONENT(RttGazeboRobotInterface)
