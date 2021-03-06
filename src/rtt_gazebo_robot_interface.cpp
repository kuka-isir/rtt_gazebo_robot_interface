#include "rtt_gazebo_robot_interface/rtt_gazebo_robot_interface.h"
#include <cmath>
using namespace RTT;

RttGazeboRobotInterface::RttGazeboRobotInterface(const std::string& name)
: TaskContext(name)
, model_name_(name)
{
    this->provides("command")->addPort("JointTorque",port_jnt_trq_in_);
    this->provides("state")->addPort("JointTorque",port_jnt_trq_out_);
    this->provides("state")->addPort("JointTorqueAct",port_jnt_trq_act_out_);
    this->provides("state")->addPort("JointGravityTorque",port_jnt_grav_trq_out_);
    this->provides("state")->addPort("JointPosition",port_jnt_pos_out_);
    this->provides("state")->addPort("JointVelocity",port_jnt_vel_out_);
    this->provides("state")->addPort("WorldToBase",port_world_to_base_out_);
    this->provides("state")->addPort("BaseVelocity",port_base_vel_out_);
    this->provides("state")->addPort("Gravity",port_gravity_out_);

    this->provides("state")->addOperation("print",&RttGazeboRobotInterface::printState,this,RTT::OwnThread);
    this->addProperty("model_name",model_name_).doc("The name of the model to load");
    this->addOperation("setModelConfiguration",&RttGazeboRobotInterface::setModelConfiguration,this,RTT::OwnThread);
    this->addOperation("setURDFString",&RttGazeboRobotInterface::setURDFString,this,RTT::OwnThread);
    this->addOperation("enableGravityCompensation",&RttGazeboRobotInterface::enableGravityCompensation,this,RTT::OwnThread);

    this->provides("world")->addOperation("getGravity",&RttGazeboRobotInterface::getGravity,this,RTT::OwnThread);
    this->addOperation("getNrOfDegreesOfFreedom",&RttGazeboRobotInterface::getNrOfDegreesOfFreedom,this,RTT::OwnThread);
}

int RttGazeboRobotInterface::getNrOfDegreesOfFreedom()
{
    return joint_map_.size();
}

void RttGazeboRobotInterface::enableGravityCompensation(bool enable_gravity_compensation)
{
    this->grav_comp_ = enable_gravity_compensation;
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

void RttGazeboRobotInterface::setURDFString(const std::string& urdf_str)
{
    this->urdf_str_ = urdf_str;
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
#if GAZEBO_MAJOR_VERSION > 8
    gazebo_model_ = world->ModelByName( model_name_ );
#else
    gazebo_model_ = world->GetModel( model_name_ );
#endif
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
        bool added = false;
        if( true
#if GAZEBO_MAJOR_VERSION >= 7
            && joint->GetType() != gazebo::physics::Joint::FIXED_JOINT
#endif
#if GAZEBO_MAJOR_VERSION > 8
            && joint->LowerLimit(0u) != joint->UpperLimit(0u)
            && joint->LowerLimit(0u) != 0
            && !std::isnan(joint->LowerLimit(0u))
            && !std::isnan(joint->UpperLimit(0u))
#else
            && joint->GetLowerLimit(0u) != joint->GetUpperLimit(0u)
            && joint->GetLowerLimit(0u).Radian() != 0
            && !std::isnan(joint->GetLowerLimit(0u).Radian())
            && !std::isnan(joint->GetUpperLimit(0u).Radian())
#endif
        )
        {

            joint_map_.push_back(joint->GetName());
            added = true;
        }
        log(Info) << "[" << getName() << "] " << (added ? "Adding":"Not adding") 
            << " joint " << joint->GetName() 
            << " type " << joint->GetType()
#if GAZEBO_MAJOR_VERSION > 8
            << " lower limit " << joint->LowerLimit(0u)
            << " upper limit " << joint->UpperLimit(0u)
#else
            << " lower limit " << joint->GetLowerLimit(0u)
            << " upper limit " << joint->GetUpperLimit(0u)
#endif
            << endlog();
    }

    if(joint_map_.size() == 0)
    {
        log(Error) << "[" << getName() << "] " << "Could not get any actuated joint for model " << model_name_ << endlog();
        return false;
    }

    std::cout << "[" << getName() << "] "<< "Physical joints" << std::endl;
    for(auto joint_name : joint_map_)
    {
        std::cout << "   - " << joint_name << std::endl;
    }

    const int ndof = joint_map_.size();

    current_jnt_pos_.setZero(ndof);
    current_jnt_vel_.setZero(ndof);
    current_jnt_trq_.setZero(ndof);
    jnt_trq_command_.setZero(ndof);

    kdl_joints_.resize(ndof);
    kdl_joint_gravity_.resize(ndof);
    KDL::SetToZero(kdl_joint_gravity_);


    if(!urdf_str_.empty())
    {
        log(Info) << "[" << getName() << "] " << "Creating KDL Tree" << endlog();
        if (!kdl_parser::treeFromString(urdf_str_, kdl_tree_))
        {
            log(Error) << "[" << getName() << "] " << "Failed to construct kdl tree" << endlog();
            return false;
        }


        KDL::Chain kdl_chain;

        const KDL::SegmentMap& segments(kdl_tree_.getSegments());

        auto gazebo_links = gazebo_model_->GetLinks();

        std::string root_link = gazebo_links[0]->GetName();
        std::string tip_link = gazebo_links[ gazebo_links.size() - 1 ]->GetName();

        if(!kdl_tree_.getChain(root_link, tip_link, kdl_chain))
        {
            log(Error) << "[" << getName() << "] " << "Failed to build the KDL chain with params :\n  root_link: [" << root_link.c_str() << "]\n  tip_link: [" << tip_link.c_str() << "]" << endlog();
            return false;
        }
#if GAZEBO_MAJOR_VERSION > 8
        auto grav = world->Gravity();
#else
        auto grav = world->GetPhysicsEngine()->GetGravity();
#endif
        KDL::Vector kdl_grav(grav[0],grav[1],grav[2]);

        grav_solver_ = std::make_shared<KDL::ChainDynParam>(kdl_chain,kdl_grav);
        log(Info) << "[" << getName() << "] " << "Created KDL chain with " << kdl_chain.getNrOfJoints() << " joints, with params :\n  root_link: [" << root_link.c_str() << "]\n  tip_link: [" << tip_link.c_str() << "]" << endlog();
    }


    log(Info) << "[" << getName() << "] " << "Connecting Gazebo events" << endlog();
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
            joint->SetForce(0,jnt_trq_command_[i] + (grav_comp_ ? kdl_joint_gravity_(i) : 0.0 ) );
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
#if GAZEBO_MAJOR_VERSION > 8
    auto world = gazebo::physics::WorldPtr();
#else
    auto world = gazebo::physics::get_world();
#endif
    for(int i=0 ; i < joint_map_.size() ; ++i)
    {
        auto joint = gazebo_model_->GetJoint( joint_map_[i] );
#if GAZEBO_MAJOR_VERSION > 8
        current_jnt_pos_[i] = joint->Position(0);
#else
        current_jnt_pos_[i] = joint->GetAngle(0).Radian();
#endif
        current_jnt_vel_[i] = joint->GetVelocity(0);
        current_jnt_trq_[i] = joint->GetForce(0u); // WARNING: This is the external estimated force

        kdl_joints_(i) = current_jnt_pos_[i];
    }
    
    grav_solver_->JntToGravity(kdl_joints_,kdl_joint_gravity_);
    current_jnt_grav_trq_ = kdl_joint_gravity_.data;
    
#if GAZEBO_MAJOR_VERSION > 8
    auto pose = gazebo_model_->RelativePose();
    current_world_to_base_.translation() = Eigen::Vector3d(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z());
    current_world_to_base_.linear() = Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z()).toRotationMatrix();

    auto base_vel_lin = gazebo_model_->RelativeLinearVel();
    auto base_vel_ang = gazebo_model_->RelativeAngularVel();
    current_base_vel_[0] = base_vel_lin[0];
    current_base_vel_[1] = base_vel_lin[1];
    current_base_vel_[2] = base_vel_lin[2];
    current_base_vel_[3] = base_vel_ang[0];
    current_base_vel_[4] = base_vel_ang[1];
    current_base_vel_[5] = base_vel_ang[2];
#else
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
#endif
    
#if GAZEBO_MAJOR_VERSION > 8
    auto grav = world->Gravity();
#else
    auto grav = world->GetPhysicsEngine()->GetGravity();
#endif
    
    gravity_[0] = grav[0];
    gravity_[1] = grav[1];
    gravity_[2] = grav[2];

    port_gravity_out_.write(gravity_);
    port_base_vel_out_.write(current_base_vel_);
    port_world_to_base_out_.write(current_world_to_base_.matrix());
    port_jnt_pos_out_.write(current_jnt_pos_);
    port_jnt_vel_out_.write(current_jnt_vel_);
    port_jnt_trq_out_.write(jnt_trq_command_);
    port_jnt_trq_act_out_.write(jnt_trq_command_ - current_jnt_grav_trq_);
    port_jnt_grav_trq_out_.write(current_jnt_grav_trq_);

    //std::cout << "GZ current_jnt_trq_      " << current_jnt_trq_.transpose() << std::endl;
    //std::cout << "GZ current_jnt_trq_qct_  " << (current_jnt_trq_ - current_jnt_grav_trq_).transpose() << std::endl;
    //std::cout << "GZ current_jnt_grav_trq_ " << current_jnt_grav_trq_.transpose() << std::endl;
}

void RttGazeboRobotInterface::updateHook()
{

}

ORO_CREATE_COMPONENT(RttGazeboRobotInterface)
