#include <at_work_robot_example_ros/robot_example_ros.h>


RobotExampleROS::RobotExampleROS(const ros::NodeHandle &nh):
    nh_(nh), seq_(0), 
    peer_public_(NULL),
    peer_team_(NULL)
{
    readParameters();

    //Publishers
    attention_message_pub_ = nh_.advertise<at_work_robot_example_ros::AttentionMessage> (
                            "attention_message", 10);

    benchmark_state_pub_ = nh_.advertise<at_work_robot_example_ros::BenchmarkState> (
                            "benchmark_state", 10);

    drill_machine_status_pub_ = nh_.advertise<at_work_robot_example_ros::DrillingMachineStatus> (
                            "drill_machine_status", 10);

    conveyor_belt_status_pub_ = nh_.advertise<at_work_robot_example_ros::TriggeredConveyorBeltStatus> (
                        "conveyor_belt_status", 10);

    inventory_pub_ = nh_.advertise<at_work_robot_example_ros::Inventory> ("inventory", 10);

    order_info_pub_ = nh_.advertise<at_work_robot_example_ros::OrderInfo> ("order_info", 10);


    //Subscribers
    drillling_machine_command_sub_ = nh_.subscribe<at_work_robot_example_ros::DrillingMachineCommand>(
                        "drillling_machine_command", 1000, &RobotExampleROS::DrillingMachineCommandCB, this);

    conveyor_belt_command_sub_ = nh_.subscribe<at_work_robot_example_ros::TriggeredConveyorBeltCommand>(
                        "conveyor_belt_command", 1000, &RobotExampleROS::TriggeredConveyorBeltCommandCB, this);

    benchmark_feedback_sub_ = nh_.subscribe<at_work_robot_example_ros::BenchmarkFeedback>(
                        "benchmark_feedback", 1000, &RobotExampleROS::BenchmarkFeedbackCB, this);

    initializeRobot();
}

RobotExampleROS::~RobotExampleROS()
{
    // Delete all global objects allocated by libprotobuf
    google::protobuf::ShutdownProtobufLibrary();
}

void RobotExampleROS::DrillingMachineCommandCB(at_work_robot_example_ros::DrillingMachineCommand msg)
{
    //create a new message
    std::shared_ptr<DrillingMachineCommand> drill_machine_command(new DrillingMachineCommand());

    rockin_msgs::DrillingMachineCommand_Command cmd = drill_machine_command->command();

    cmd = (rockin_msgs::DrillingMachineCommand_Command)msg.command.data;

    drill_machine_command->set_command(cmd);

    //send the Message over team peer
    peer_team_->send(drill_machine_command);
}

void RobotExampleROS::TriggeredConveyorBeltCommandCB(at_work_robot_example_ros::TriggeredConveyorBeltCommand msg)
{
    //create a new message
    std::shared_ptr<TriggeredConveyorBeltCommand> conveyor_belt_command(new TriggeredConveyorBeltCommand);

    //fill the message
    rockin_msgs::ConveyorBeltRunMode cmd = conveyor_belt_command->command();

    cmd = (rockin_msgs::ConveyorBeltRunMode)msg.command.data;

    conveyor_belt_command->set_command(cmd);

    conveyor_belt_command->set_next_cycle(msg.next_cycle.data);

    //send the Message over team peer
    peer_team_->send(conveyor_belt_command);
}

void RobotExampleROS::BenchmarkFeedbackCB(at_work_robot_example_ros::BenchmarkFeedback msg)
{
    //create a new message
    std::shared_ptr<BenchmarkFeedback> benchmark_feedback(new BenchmarkFeedback);

    //fill the message for FBM1
    benchmark_feedback->set_object_class_name((std::string)msg.object_class_name.data);

    rockin_msgs::Pose3D *pose_object = benchmark_feedback->mutable_object_pose();

    rockin_msgs::Position3D *position_object =  pose_object->mutable_position();
    rockin_msgs::Quaternion *orientation_object =  pose_object->mutable_orientation();

    position_object->set_x(msg.object_pose.position.x);
    position_object->set_y(msg.object_pose.position.y);
    position_object->set_z(msg.object_pose.position.z);

    orientation_object->set_x(msg.object_pose.orientation.x);
    orientation_object->set_y(msg.object_pose.orientation.y);
    orientation_object->set_z(msg.object_pose.orientation.z);
    orientation_object->set_w(msg.object_pose.orientation.w);

    //fill the message for FBM1+FMB2
    benchmark_feedback->set_object_instance_name((std::string)msg.object_instance_name.data);

    //fill the message for FBM2
    benchmark_feedback->set_grasp_notification(msg.grasp_notification.data);
    rockin_msgs::Pose3D *pose_eef = benchmark_feedback->mutable_end_effector_pose();

    rockin_msgs::Position3D *position_eef =  pose_eef->mutable_position();
    rockin_msgs::Quaternion *orientation_eef =  pose_eef->mutable_orientation();

    position_eef->set_x(msg.end_effector_pose.position.x);
    position_eef->set_y(msg.end_effector_pose.position.y);
    position_eef->set_z(msg.end_effector_pose.position.z);

    orientation_eef->set_x(msg.end_effector_pose.orientation.x);
    orientation_eef->set_y(msg.end_effector_pose.orientation.y);
    orientation_eef->set_z(msg.end_effector_pose.orientation.z);
    orientation_eef->set_w(msg.end_effector_pose.orientation.w);

    // make sure we have a valid plate state (if it wasn't set earlier)
    if (msg.plate_state_after_receiving.data == 0)
    {
        msg.plate_state_after_receiving.data = BenchmarkFeedback::UNUSABLE;
    }
    if (msg.plate_state_after_drilling.data == 0)
    {
        msg.plate_state_after_drilling.data = BenchmarkFeedback::PERFECT;
    }

    //fill the message for TBM1
    benchmark_feedback->set_assembly_aid_tray_id((std::string)msg.assembly_aid_tray_id.data);
    benchmark_feedback->set_container_id((std::string)msg.container_id.data);

    ////fill the message TBM2
    rockin_msgs::BenchmarkFeedback_PlateState plate_state = benchmark_feedback->after_receiving();
    plate_state = (rockin_msgs::BenchmarkFeedback_PlateState)msg.plate_state_after_receiving.data;
    benchmark_feedback->set_after_receiving(plate_state);

    plate_state = benchmark_feedback->after_drilling();
    plate_state = (rockin_msgs::BenchmarkFeedback_PlateState)msg.plate_state_after_drilling.data;
    benchmark_feedback->set_after_drilling(plate_state);

    //send the Message over team peer
    peer_team_->send(benchmark_feedback);
}


void RobotExampleROS::readParameters()
{
    ros::param::param<bool>("~remote_refbox", remote_refbox_, false);
    ros::param::param<std::string>("~host_name", host_name_, "localhost");

    //Paramters to use when ref box is running on remote machine.
    ros::param::param<int>("~public_port", public_port_, 4444);
    ros::param::param<int>("~team_port", team_port_, 4452);

    //Paramters to use when ref box is running on same machine as client.
    ros::param::param<int>("~refbox_send_port", public_recv_port_ , 4444);
    ros::param::param<int>("~refbox_recv_port", public_send_port_, 4445);
    ros::param::param<int>("~team_send_port", team_send_port_, 4453);
    ros::param::param<int>("~team_recv_port", team_recv_port_, 4452);

    ros::param::param<std::string>("~robot_name", robot_name_, "spqr");
    ros::param::param<std::string>("~team_name", team_name_, "SPQR");

    ROS_INFO("Hostname: %s", host_name_.c_str());
    ROS_INFO("Team Port: %i", team_port_);
    ROS_INFO("Public Port: %i", public_port_);
    ROS_INFO("Name: %s", robot_name_.c_str());
    ROS_INFO("Team Name: %s", team_name_.c_str());
}

void RobotExampleROS::initializeRobot()
{
    //create public peer
    if (remote_refbox_) {
        //ref box is running on remote machine.
        peer_public_.reset(new ProtobufBroadcastPeer(host_name_, 
                                                    public_port_));
    } else {
        //ref box is running on same machine as client.
        peer_public_.reset(new ProtobufBroadcastPeer(host_name_, 
                                                    public_send_port_, 
                                                    public_recv_port_));    
    }
    

    //create internal message handler
    MessageRegister &message_register = peer_public_->message_register();
    //added messagetype to the handler
    message_register.add_message_type<AttentionMessage>();
    message_register.add_message_type<BeaconSignal>();
    message_register.add_message_type<BenchmarkState>();
    message_register.add_message_type<BenchmarkFeedback>();
    message_register.add_message_type<Inventory>();
    message_register.add_message_type<OrderInfo>();
    message_register.add_message_type<RobotInfo>();
    message_register.add_message_type<VersionInfo>();
    message_register.add_message_type<DrillingMachineStatus>();
    message_register.add_message_type<TriggeredConveyorBeltStatus>();
    message_register.add_message_type<DrillingMachineCommand>();
    message_register.add_message_type<TriggeredConveyorBeltCommand>();

    //create team peer and linked to internal message handler
    if (remote_refbox_) {
        //ref box is running on remote machine.
        peer_team_.reset(new ProtobufBroadcastPeer(host_name_,
                                                    team_port_, 
                                                    &message_register));
    } else {
        //ref box is running on same machine as client.
        peer_team_.reset(new ProtobufBroadcastPeer(host_name_,
                                                    team_send_port_, 
                                                    team_recv_port_, 
                                                    &message_register));
    }

    //bind the peers to the callback funktions
    peer_public_->signal_received().connect(boost::bind(&RobotExampleROS::handleMessage, this, _1, _2, _3, _4));
    peer_public_->signal_send_error().connect(boost::bind( &RobotExampleROS::handleSendError, this, _1));
    peer_public_->signal_recv_error().connect(boost::bind(&RobotExampleROS::handleReceiveError, this, _1, _2));

    peer_team_->signal_received().connect(boost::bind(&RobotExampleROS::handleMessage, this, _1, _2, _3, _4));
    peer_team_->signal_send_error().connect(boost::bind( &RobotExampleROS::handleSendError, this, _1));
    peer_team_->signal_recv_error().connect(boost::bind(&RobotExampleROS::handleReceiveError, this, _1, _2));
}

void RobotExampleROS::sendBeacon()
{
    //generate the timestamp
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    int32_t sec = start.tv_sec;
    int32_t nsec = start.tv_nsec;

    //create the message
    std::shared_ptr<BeaconSignal> signal(new BeaconSignal());

    //seperate the time segment of the message
    Time *time = signal->mutable_time(); 
    //write the timestamp into the message
    time->set_sec(sec);
    time->set_nsec(nsec);

    //write the name and sequence counter into the message
    signal->set_peer_name(robot_name_);
    signal->set_team_name(team_name_);
    //increase the sequence number
    signal->set_seq(++seq_);
    //send over team peer       
    peer_team_->send(signal);
}

void RobotExampleROS::handleSendError(std::string msg)
{
    ROS_WARN("Send error: %s\n", msg.c_str());
}

void RobotExampleROS::handleReceiveError(boost::asio::ip::udp::endpoint &endpoint,
                        std::string msg)
{
    ROS_WARN("Recv error: %s\n", msg.c_str());
}

void RobotExampleROS::handleMessage(boost::asio::ip::udp::endpoint &sender,
                    uint16_t component_id, uint16_t msg_type,
                    std::shared_ptr<google::protobuf::Message> msg)
{
    std::shared_ptr<AttentionMessage> attention_msg_ptr;

    std::shared_ptr<BenchmarkState> benchmark_state_ptr;

    std::shared_ptr<DrillingMachineStatus> drill_machine_status_ptr;

    std::shared_ptr<TriggeredConveyorBeltStatus> conveyor_belt_status_ptr;

    std::shared_ptr<Inventory> inventory_pub_ptr;

    std::shared_ptr<OrderInfo> order_info_ptr;
             
    if ((attention_msg_ptr = std::dynamic_pointer_cast<AttentionMessage>(msg))) {

        at_work_robot_example_ros::AttentionMessage attention_msg;

        attention_msg.message.data      = attention_msg_ptr->message();
        attention_msg.time_to_show.data = attention_msg_ptr->time_to_show();
        attention_msg.team.data         = attention_msg_ptr->team();

        attention_message_pub_.publish(attention_msg);

    } else if ((benchmark_state_ptr = std::dynamic_pointer_cast<BenchmarkState>(msg))) {

        at_work_robot_example_ros::BenchmarkState benchmark_state_msg;

        benchmark_state_msg.benchmark_time.data.sec =
                                        benchmark_state_ptr->benchmark_time().sec();
        benchmark_state_msg.benchmark_time.data.nsec =
                                        benchmark_state_ptr->benchmark_time().nsec();
        benchmark_state_msg.state.data =
                                        benchmark_state_ptr->state();
        benchmark_state_msg.phase.data =
                                        benchmark_state_ptr->phase();
        benchmark_state_msg.scenario.type.data =
                                        benchmark_state_ptr->scenario().type();
        benchmark_state_msg.scenario.type_id.data =
                                        benchmark_state_ptr->scenario().type_id();
        benchmark_state_msg.scenario.description.data =
                                        benchmark_state_ptr->scenario().description();

        benchmark_state_msg.known_teams.resize(benchmark_state_ptr->known_teams().size());

        for(int i=0; i < benchmark_state_ptr->known_teams().size(); i++) {
            benchmark_state_msg.known_teams[i].data =
                                        benchmark_state_ptr->known_teams(i);
        }

        benchmark_state_msg.connected_teams.resize(benchmark_state_ptr->connected_teams().size());

        for(int i=0; i < benchmark_state_ptr->connected_teams().size(); i++) {
            benchmark_state_msg.connected_teams[i].data =
                                        benchmark_state_ptr->connected_teams(i);
        }

        benchmark_state_pub_.publish(benchmark_state_msg);

    } else if ((drill_machine_status_ptr = std::dynamic_pointer_cast<DrillingMachineStatus>(msg))) {

        at_work_robot_example_ros::DrillingMachineStatus drill_machine_msg;

        drill_machine_msg.state.data = drill_machine_status_ptr->state();

        drill_machine_status_pub_.publish(drill_machine_msg);

    } else if ((conveyor_belt_status_ptr = std::dynamic_pointer_cast<TriggeredConveyorBeltStatus>(msg))) {

        at_work_robot_example_ros::TriggeredConveyorBeltStatus conveyor_belt_status_msg;

        conveyor_belt_status_msg.state.data = conveyor_belt_status_ptr->state();

        conveyor_belt_status_msg.cycle.data = conveyor_belt_status_ptr->cycle();

        conveyor_belt_status_pub_.publish(conveyor_belt_status_msg);

    } else if ((inventory_pub_ptr = std::dynamic_pointer_cast<Inventory>(msg))) {

        at_work_robot_example_ros::Inventory inventory_msg;

        inventory_msg.items.resize(inventory_pub_ptr->items().size());

        for(int i=0; i < inventory_pub_ptr->items().size(); i++) {

            inventory_msg.items[i].object.type.data =
                                        inventory_pub_ptr->items(i).object().type();

            inventory_msg.items[i].object.type_id.data =
                                        inventory_pub_ptr->items(i).object().type_id();

            inventory_msg.items[i].object.instance_id.data =
                                        inventory_pub_ptr->items(i).object().instance_id();

            inventory_msg.items[i].object.description.data =
                                        inventory_pub_ptr->items(i).object().description();

            inventory_msg.items[i].quantity.data =
                                        inventory_pub_ptr->items(i).quantity();

            inventory_msg.items[i].container.type.data =
                                        inventory_pub_ptr->items(i).container().type();

            inventory_msg.items[i].container.type_id.data =
                                        inventory_pub_ptr->items(i).container().type_id();

            inventory_msg.items[i].container.instance_id.data =
                                        inventory_pub_ptr->items(i).container().instance_id();

            inventory_msg.items[i].container.description.data =
                                        inventory_pub_ptr->items(i).container().description();

            inventory_msg.items[i].location.type.data =
                                        inventory_pub_ptr->items(i).location().type();

            inventory_msg.items[i].location.instance_id.data =
                                        inventory_pub_ptr->items(i).location().instance_id();

            inventory_msg.items[i].location.description.data =
                                        inventory_pub_ptr->items(i).location().description();
        }

        inventory_pub_.publish(inventory_msg);

    }  else if ((order_info_ptr = std::dynamic_pointer_cast<OrderInfo>(msg))) {

        at_work_robot_example_ros::OrderInfo order_info_msg;

        order_info_msg.orders.resize(order_info_ptr->orders().size());

        for(int i=0; i < order_info_ptr->orders().size(); i++) {

            order_info_msg.orders[i].id.data =
                                        order_info_ptr->orders(i).id();
            order_info_msg.orders[i].status.data =
                                        order_info_ptr->orders(i).status();

            order_info_msg.orders[i].object.type.data =
                                        order_info_ptr->orders(i).object().type();

            order_info_msg.orders[i].object.type_id.data =
                                        order_info_ptr->orders(i).object().type_id();

            order_info_msg.orders[i].object.instance_id.data =
                                        order_info_ptr->orders(i).object().instance_id();

            order_info_msg.orders[i].object.description.data =
                                        order_info_ptr->orders(i).object().description();

            order_info_msg.orders[i].container.type.data =
                                        order_info_ptr->orders(i).container().type();

            order_info_msg.orders[i].container.type_id.data =
                                        order_info_ptr->orders(i).container().type_id();

            order_info_msg.orders[i].container.instance_id.data =
                                        order_info_ptr->orders(i).container().instance_id();

            order_info_msg.orders[i].container.description.data =
                                        order_info_ptr->orders(i).container().description();

            order_info_msg.orders[i].quantity_delivered.data =
                                        order_info_ptr->orders(i).quantity_delivered();

            order_info_msg.orders[i].quantity_requested.data =
                                        order_info_ptr->orders(i).quantity_requested();

            order_info_msg.orders[i].destination.type.data =
                                        order_info_ptr->orders(i).destination().type();

            order_info_msg.orders[i].destination.instance_id.data =
                                        order_info_ptr->orders(i).destination().instance_id();

            order_info_msg.orders[i].destination.description.data =
                                        order_info_ptr->orders(i).destination().description();

            order_info_msg.orders[i].source.type.data =
                                        order_info_ptr->orders(i).source().type();

            order_info_msg.orders[i].source.instance_id.data =
                                        order_info_ptr->orders(i).source().instance_id();

            order_info_msg.orders[i].source.description.data =
                                        order_info_ptr->orders(i).source().description();

            order_info_msg.orders[i].processing_team.data =
                                        order_info_ptr->orders(i).processing_team();
        }

        order_info_pub_.publish(order_info_msg);

    }
}
