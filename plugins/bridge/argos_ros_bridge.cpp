/*
 * argos_ros_footbot.cpp
 *
 *  Created on: 20 Jun 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

/* Include the controller definition */
#include "argos_ros_bridge.h"
#include <argos3/core/utility/math/angles.h>
using namespace std;
using namespace argos3_ros2_bridge;
using namespace argos3_ros2_bridge::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

bool ArgosRosBridge::ros_initialized = false;
rclcpp::Context::SharedPtr ArgosRosBridge::global_context_ = nullptr;

ArgosRosBridge::ArgosRosBridge() : robot_id_(),
								   multiple_domains_(false),
								   nodes_per_domain_(0),
								   domain_id_(0),
								   enable_time_synchronization(false),
								   max_attempts(10),
								   nodeHandle_(nullptr),
								   // --- publishers ---
								   lightListPublisher_(nullptr),
								   promixityListPublisher_(nullptr),
								   blobListPublisher_(nullptr),
								   positionPublisher_(nullptr),
								   rabPublisher_(nullptr),
								   clockPublisher_(nullptr),
								   // --- subscribers ---
								   cmdVelSubscriber_(nullptr),
								   cmdRabSubscriber_(nullptr),
								   cmdLedSubscriber_(nullptr),
								   // --- timer ---
								   timer_(nullptr),
								   // --- actuators / sensors ---
								   m_pcWheels(nullptr),
								   m_pcLight(nullptr),
								   m_pcLEDs(nullptr),
								   m_pcCamera(nullptr),
								   m_pcProximity(nullptr),
								   m_pcPosition(nullptr),
								   m_pcRABS(nullptr),
								   m_pcRABA(nullptr),
								   // --- params / counters ---
								   stopWithoutSubscriberCount(10),
								   stepsSinceCallback(0),
								   leftSpeed(0.0),
								   rightSpeed(0.0)
{
}

ArgosRosBridge::~ArgosRosBridge() {}

void ArgosRosBridge::Init(TConfigurationNode &t_node)
{

	// Get robot ID from ARGoS (e.g., "bot0", "bot1")
	robot_id_ = GetId();
	GetNodeAttributeOrDefault(t_node, "multiple_domains", multiple_domains_, false);
	GetNodeAttributeOrDefault(t_node, "nodes_per_domain", nodes_per_domain_, 50);
	GetNodeAttributeOrDefault(t_node, "ros_domain_id", domain_id_, 0);
	GetNodeAttributeOrDefault(t_node, "half_baseline", halfBaseline_, halfBaseline_);
	GetNodeAttributeOrDefault(t_node, "wheel_radius", wheelRadius_, wheelRadius_);

	// Calculate domain ID from robot ID
	if (multiple_domains_)
	{
		std::string bot_prefix = "bot";
		if (robot_id_.find(bot_prefix) == 0)
		{
			std::string num_str = robot_id_.substr(bot_prefix.length());
			try
			{
				int robot_num = std::stoi(num_str);
				domain_id_ = robot_num / nodes_per_domain_; // Group by nodes_per_domain_
			}
			catch (const std::exception &e)
			{
				RCLCPP_ERROR(rclcpp::get_logger("argos"), "Invalid robot ID format: %s", robot_id_.c_str());
				domain_id_ = 0;
			}
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger("argos"), "Robot ID %s doesn't start with 'bot'", robot_id_.c_str());
			domain_id_ = 0;
		}
	}

	// Create a context with the domain ID
	auto context = std::make_shared<rclcpp::Context>();
	rclcpp::InitOptions init_options;
	init_options.set_domain_id(domain_id_);
	init_options.auto_initialize_logging(false); // Prevent multiple logging inits
	context->init(0, nullptr, init_options);

	// Create node with unique name
	std::string node_name = "argos_ros_node_" + robot_id_;
	nodeHandle_ = std::make_shared<rclcpp::Node>(node_name, rclcpp::NodeOptions().context(context));

	// Retrieve and print the actual domain ID
	auto actual_domain_id = context->get_domain_id();
	RCLCPP_INFO(nodeHandle_->get_logger(), "Running on ROS_DOMAIN_ID: %zu", actual_domain_id);

	/********************************
	 * For clock
	 ********************************/
	if (enable_time_synchronization)
	{
		// 1. Create the topics to publish argos3 clock
		std::stringstream argosClockTopic;
		argosClockTopic << "/" << robot_id_ << "/argos3_clock";
		clockPublisher_ = nodeHandle_->create_publisher<rosgraph_msgs::msg::Clock>(argosClockTopic.str(), 1);

		// 2. Create the subscribers to subscribe the ros2 clock
		std::stringstream rosClockTopic;
		rosClockTopic << "/" << robot_id_ << "/ros2_clock";
		clockSubscriber_ = nodeHandle_->create_subscription<rosgraph_msgs::msg::Clock>(
			rosClockTopic.str(),
			1,
			std::bind(&ArgosRosBridge::clockCallback, this, _1));
	}
	/********************************
	 * For the robot sensors:
	 * 1. Get sensor handles
	 * 2. Create the topics to publish
	 ********************************/
	if (HasSensor("footbot_light"))
	{
		stringstream lightTopic;
		lightTopic << "/" << robot_id_ << "/lightList";
		m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
		lightListPublisher_ = nodeHandle_->create_publisher<LightList>(lightTopic.str(), 1);
	}
	if (HasSensor("footbot_proximity"))
	{
		stringstream proxTopic;
		proxTopic << "/" << robot_id_ << "/proximityList";
		m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
		promixityListPublisher_ = nodeHandle_->create_publisher<ProximityList>(proxTopic.str(), 1);
	}
	if (HasSensor("positioning"))
	{
		stringstream positionTopic;
		positionTopic 		<< "/" << robot_id_ << "/position";
		stringstream odomTopic;
		odomTopic			<< "/" << robot_id_ << "/odom";
		m_pcPosition 		= GetSensor < CCI_PositioningSensor>("positioning");
		positionPublisher_ 	= nodeHandle_ -> create_publisher<Position>(positionTopic.str(), 1);
		odometryPublisher_	= nodeHandle_->create_publisher<nav_msgs::msg::Odometry>(odomTopic.str(), 1);
	}
	if (HasSensor("range_and_bearing"))
	{
		stringstream rabTopic;
		rabTopic << "/" << robot_id_ << "/rab";
		m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
		rabPublisher_ = nodeHandle_->create_publisher<PacketList>(rabTopic.str(), 1);
	}
	if (HasSensor("colored_blob_omnidirectional_camera"))
	{
		stringstream blobTopic;
		blobTopic << "/" << robot_id_ << "/blobList";
		m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
		blobListPublisher_ = nodeHandle_->create_publisher<BlobList>(blobTopic.str(), 1);
	}
	if (HasSensor("turtlebot3_lidar")){
		stringstream lidarTopic;
		lidarTopic 			<< "/" << robot_id_ << "/lidarList";
		stringstream lidarScanTopic;
		lidarScanTopic		<< "/" << robot_id_ << "/scan";
		m_pcLidar 			= GetSensor < CCI_Turtlebot3LIDARSensor>("turtlebot3_lidar");
		lidarPublisher_ 	= nodeHandle_ -> create_publisher<LidarList>(lidarTopic.str(), 1);
		lidarScanPublisher_	= nodeHandle_ -> create_publisher<sensor_msgs::msg::LaserScan>(lidarScanTopic.str(), 1);
	}
	if (HasSensor("differential_steering")){
		stringstream wheelVelTopic;
		wheelVelTopic 		<< "/" << robot_id_ << "/wheelVelocities";
		m_pcWheelsSensor 	= GetSensor < CCI_DifferentialSteeringSensor>("differential_steering");
		wheelVelocitiesPublisher_ 	= nodeHandle_ -> create_publisher<WheelVelocities>(wheelVelTopic.str(), 1);
	}

	/********************************
	 * For the robot actuators:
	 * 1. Get actuator handles
	 * 2. Create the subscribers to subscribe to the topics
	 ********************************/
	if (HasActuator("leds"))
	{
		m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
		stringstream cmdLedTopic;
		cmdLedTopic << "/" << robot_id_ << "/cmd_led";
		cmdLedSubscriber_ = nodeHandle_->create_subscription<Led>(
			cmdLedTopic.str(),
			1,
			std::bind(&ArgosRosBridge::cmdLedCallback, this, _1));
	}
	if (HasActuator("range_and_bearing"))
	{
		m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
		stringstream cmdRabTopic;
		cmdRabTopic << "/" << robot_id_ << "/cmd_rab";
		cmdRabSubscriber_ = nodeHandle_->create_subscription<Packet>(
			cmdRabTopic.str(),
			1,
			std::bind(&ArgosRosBridge::cmdRabCallback, this, _1));
	}

	if (HasActuator("differential_steering"))
	{
		m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
		stringstream cmdVelTopic;
		cmdVelTopic << "/" << robot_id_ << "/cmd_vel";
		cmdVelSubscriber_ = nodeHandle_->create_subscription<Twist>(
			cmdVelTopic.str(),
			1,
			std::bind(&ArgosRosBridge::cmdVelCallback, this, _1));
	}

	/*
	 * Other init stuff
	 */
	if (HasSensor("colored_blob_omnidirectional_camera"))
	{
		/* Enable camera filtering */
		m_pcCamera->Enable();
	}
	if (HasActuator("leds"))
	{
		/* Enable */
		m_pcLEDs->SetSingleColor(12, CColor::RED);
	}
	/*
	 * Parse the configuration file
	 *
	 * The user defines this part. Here, the algorithm accepts three
	 * parameters and it's nice to put them in the config file so we don't
	 * have to recompile if we want to try other settings.
	 */
	GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
	// Start spinning in a detached thread
	// std::thread([this]() { rclcpp::spin(nodeHandle_); }).detach();
}

bool blobComparator(Blob a, Blob b)
{
	return a.angle < b.angle;
}

void ArgosRosBridge::ControlStep()
{

	rclcpp::spin_some(nodeHandle_);

	/*********************************
	 * Publish clock
	 *********************************/
	if (enable_time_synchronization)
	{
		const auto &sim = argos::CSimulator::GetInstance();
		argos::CPhysicsEngine &engine = sim.GetPhysicsEngine("dyn2d");

		argos::Real sim_time = sim.GetSpace().GetSimulationClock() * engine.GetPhysicsClockTick();

		// Convert argos3 time (seconds) to ROS2 time (nanoseconds)
		rclcpp::Time ros_sim_time(static_cast<uint64_t>(sim_time * 1e9));

		rosgraph_msgs::msg::Clock msg;
		msg.clock = ros_sim_time;
		clockPublisher_->publish(msg);
	}

	/*********************************
	 * Get readings from light sensor
	 *********************************/
	if (HasSensor("footbot_light"))
	{
		const CCI_FootBotLightSensor::TReadings &tLightReads = m_pcLight->GetReadings();
		LightList lightList;
		lightList.n = tLightReads.size();
		for (int i = 0; i < lightList.n; ++i)
		{
			Light light;
			light.value = tLightReads[i].Value;
			light.angle = tLightReads[i].Angle.GetValue();
			lightList.lights.push_back(light);
		}

		lightListPublisher_->publish(lightList);
	}
	/***********************************
	 * Get readings from footbot proximity sensor
	 ***********************************/
	if (HasSensor("footbot_proximity"))
	{
		const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
		ProximityList proxList;
		proxList.n = tProxReads.size();
		for (int i = 0; i < proxList.n; ++i)
		{
			Proximity prox;
			prox.value = tProxReads[i].Value;
			prox.angle = tProxReads[i].Angle.GetValue();
			proxList.proximities.push_back(prox);
		}

		promixityListPublisher_->publish(proxList);
	}
	/***********************************
	 * Get readings from turtlebot 3 proximity sensor
	 ***********************************/
	if (HasSensor("turtlebot3_proximity")){
		const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
		ProximityList proxList;
		proxList.n = tProxReads.size();
		for (size_t i = 0; i < proxList.n; ++i) {
			Proximity prox;
			prox.value = tProxReads[i].Value;
			prox.angle = tProxReads[i].Angle.GetValue();
			proxList.proximities.push_back(prox);

		}

		promixityListPublisher_ -> publish(proxList);
	}
	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
	if (HasSensor("colored_blob_omnidirectional_camera"))
	{
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &camReads = m_pcCamera->GetReadings();
		BlobList blobList;
		blobList.n = camReads.BlobList.size();
		Blob blob;
		for (int i = 0; i < blobList.n; ++i)
		{
			// Blob blob;
			stringstream ss;
			ss << camReads.BlobList[i]->Color;
			blob.color = ss.str();
			blob.distance = camReads.BlobList[i]->Distance;

			// Make the angle of the puck in the range [-PI, PI].  This is useful for
			// tasks such as homing in on a puck using a simple controller based on
			// the sign of this angle.
			blob.angle = camReads.BlobList[i]->Angle.GetValue(); //.SignedNormalize().GetValue();
			blobList.blobs.push_back(blob);
		}

		// Sort the blob list by angle.  This is useful for the purposes of extracting meaning from
		// the local blob configuration (e.g. fitting a lines to the detected blobs).
		sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

		blobListPublisher_->publish(blobList);
	}

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
	if (HasSensor("positioning")){
		const auto now = nodeHandle_->now();
		const CCI_PositioningSensor::SReading& tPosReads = m_pcPosition->GetReading();
		Position position;

		position.position.x = tPosReads.Position.GetX();
		position.position.y = tPosReads.Position.GetY();
		position.position.z = tPosReads.Position.GetZ();

		position.orientation.w = tPosReads.Orientation.GetW();
		position.orientation.x = tPosReads.Orientation.GetX();
		position.orientation.y = tPosReads.Orientation.GetY();
		position.orientation.z = tPosReads.Orientation.GetZ();

		positionPublisher_ -> publish(position);

		if (odometryPublisher_) {
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = now;
			odom.header.frame_id = robot_id_ + "/odom";
			odom.child_frame_id = robot_id_ + "/base_link";

			odom.pose.pose.position.x = position.position.x;
			odom.pose.pose.position.y = position.position.y;
			odom.pose.pose.position.z = position.position.z;
			odom.pose.pose.orientation = position.orientation;

			odom.twist.twist.linear.x = 0.0;
			odom.twist.twist.linear.y = 0.0;
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			odom.twist.twist.angular.z = 0.0;

			odometryPublisher_->publish(odom);
		}
	}

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	if (HasSensor("range_and_bearing"))
	{
		const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
		PacketList packetList;
		packetList.n = tRabReads.size();
		for (int i = 0; i < packetList.n; ++i)
		{
			Packet packet;
			packet.range = tRabReads[i].Range;
			packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
			packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

			packet.data.push_back(tRabReads[i].Data[0]);
			packet.data.push_back(tRabReads[i].Data[1]);

			packetList.packets.push_back(packet);
		}

		rabPublisher_->publish(packetList);
	}

	/**********************************************
	 * Get readings from Turtlebot3 LiDAR sensor
	 **********************************************/
	if (HasSensor("turtlebot3_lidar")){
		const auto now = nodeHandle_->now();
		const std::string lidar_frame = robot_id_ + "/lidar";

		LidarList lidarList;
		lidarList.header.stamp = now;
		lidarList.header.frame_id = lidar_frame;
		lidarList.n = m_pcLidar->GetNumReadings();
		const size_t lidar_count = static_cast<size_t>(lidarList.n);
		if (lidar_count > 0) {
			lidarList.lidars.reserve(lidar_count);
		}

		sensor_msgs::msg::LaserScan laserScan;
		laserScan.header = lidarList.header;
		if (lidar_count > 0) {
			const double angle_increment_rad = argos::CRadians::TWO_PI.GetValue() / static_cast<double>(lidar_count);
			const double angle_increment_deg = 360.0 / static_cast<double>(lidar_count);
			laserScan.angle_min = -argos::CRadians::PI.GetValue();
			laserScan.angle_increment = angle_increment_rad;
			laserScan.angle_max = laserScan.angle_min + static_cast<double>(lidar_count - 1) * angle_increment_rad;
			laserScan.time_increment = 0.0;
			laserScan.scan_time = 0.0;
			laserScan.range_min = 0.12f;
			laserScan.range_max = 3.5f;
			laserScan.ranges.resize(lidar_count);
			laserScan.intensities.assign(lidar_count, 0.0f);

			for (size_t i = 0; i < lidar_count; ++i) {
				Lidar lidar;
				lidar.angle = static_cast<float>(i * angle_increment_deg); // angle in degrees
				lidar.value = static_cast<float>(m_pcLidar->GetReading(i));
				lidarList.lidars.push_back(lidar);

				// Convert millimetres to metres for ROS LaserScan consumers
				const float range_metres = static_cast<float>(lidar.value * 0.001f);
				laserScan.ranges[i] = range_metres;
			}
		} else {
			laserScan.angle_min = 0.0;
			laserScan.angle_max = 0.0;
			laserScan.angle_increment = 0.0;
			laserScan.range_min = 0.0;
			laserScan.range_max = 0.0;
		}

		if (lidarPublisher_) {
			lidarPublisher_->publish(lidarList);
		}
		if (lidarScanPublisher_) {
			lidarScanPublisher_->publish(laserScan);
		}
	}

	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount)
	{
		leftSpeed = 0;
		rightSpeed = 0;
	}
	else
	{
		stepsSinceCallback++;
	}
	if (HasActuator("differential_steering"))
	{
		m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
	}

	/*********************************************
	 * Time synchronization
	 *********************************************/
	if (enable_time_synchronization)
	{
		const auto &sim = argos::CSimulator::GetInstance();
		argos::CPhysicsEngine &engine = sim.GetPhysicsEngine("dyn2d");
		double argos_time_sec =
			sim.GetSpace().GetSimulationClock() * engine.GetPhysicsClockTick();

		rclcpp::Time argos_time(static_cast<int64_t>(argos_time_sec * 1e9));

		double tolerance = 1e-9;

		for (int attempt = 0; attempt < max_attempts; ++attempt)
		{
			// Compute the time difference between ROS2 clock and ARGoS simulation time
			double diff = std::abs(ros2_time_.seconds() - argos_time.seconds());

			// If within tolerance → clocks are synchronized
			if (diff <= tolerance)
			{
				break;
			}
			// Sleep briefly before checking again
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			if (attempt == max_attempts - 1)
			{
				std::cout << "Time synchronization failed after " << max_attempts << ", last diff = " << diff << ". Please increase the max_attempts." << std::endl;
			}
		}
	}
}

void ArgosRosBridge::cmdVelCallback(const Twist &twist)
{
	double v = twist.linear.x;		// Forward linear velocity
	double omega = twist.angular.z; // Rotational (angular) velocity
	double L = halfBaseline_ * 2.0;	// Distance between wheels (wheelbase)
	double R = wheelRadius_;		// Wheel radius

	// Calculate left and right wheel speeds using differential drive kinematics
	leftSpeed = (v - (L / 2) * omega) / R;
	rightSpeed = (v + (L / 2) * omega) / R;

	stepsSinceCallback = 0;
}

void ArgosRosBridge::cmdRabCallback(const Packet &packet)
{
	m_pcRABA->SetData(0, packet.data[0]);
	m_pcRABA->SetData(1, std::stoi(packet.id));
}
void ArgosRosBridge::cmdLedCallback(const Led &ledColor)
{
	/**
	 * TODO: Better way to set the led colors instead of this exhaustive if-else
	 */
	if (ledColor.color == "red")
	{
		if (ledColor.mode == "ALL")
		{
			m_pcLEDs->SetAllColors(CColor::RED);
		}
		else if (ledColor.mode == "SINGLE")
		{
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::RED);
		}
	}
	else if (ledColor.color == "yellow")
	{
		if (ledColor.mode == "ALL")
		{
			m_pcLEDs->SetAllColors(CColor::YELLOW);
		}
		else if (ledColor.mode == "SINGLE")
		{
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::YELLOW);
		}
	}
	else if (ledColor.color == "green")
	{
		if (ledColor.mode == "ALL")
		{
			m_pcLEDs->SetAllColors(CColor::GREEN);
		}
		else if (ledColor.mode == "SINGLE")
		{
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::GREEN);
		}
	}
	else if (ledColor.color == "magenta")
	{
		if (ledColor.mode == "ALL")
		{
			m_pcLEDs->SetAllColors(CColor::MAGENTA);
		}
		else if (ledColor.mode == "SINGLE")
		{
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::MAGENTA);
		}
	}
	else if (ledColor.color == "black")
	{
		if (ledColor.mode == "ALL")
		{
			m_pcLEDs->SetAllColors(CColor::BLACK);
		}
		else if (ledColor.mode == "SINGLE")
		{
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::BLACK);
		}
	}
}

void ArgosRosBridge::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
	ros2_time_ = msg->clock;
}

void ArgosRosBridge::Destroy()
{

	nodeHandle_.reset(); // Destroy node first
}
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(ArgosRosBridge, "argos_ros_bot_controller")
