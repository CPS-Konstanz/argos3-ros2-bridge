/*
 * argos_ros_footbot.cpp
 *
 *  Created on: 20 Jun 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

/* Include the controller definition */
#include "argos_ros_bridge.h"
using namespace std;
using namespace argos3_ros2_bridge;
using namespace argos3_ros2_bridge::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

/**
 * Initialize the node before creating the publishers
 * and subscribers. Otherwise we get a guard-error during
 * compilation if we initialize the node after.
 */
std::shared_ptr<rclcpp::Node> initNode() {
  int argc = 1;
  char *argv = (char *) "";
  if (rclcpp::get_contexts().empty()){rclcpp::init(argc, &argv);}
  
  return std::make_shared<rclcpp::Node>("argos_ros_node");

}

std::shared_ptr<rclcpp::Node> ArgosRosBridge::nodeHandle = initNode();

ArgosRosBridge::ArgosRosBridge() :
		m_pcWheels(NULL),
		m_pcLight(NULL),
		m_pcLEDs(NULL),
		m_pcCamera(NULL),
		m_pcProximity(NULL),
		m_pcPosition(NULL),
		m_pcRABA(NULL),
		m_pcRABS(NULL),
		stopWithoutSubscriberCount(10),
		stepsSinceCallback(0),
		leftSpeed(0),
		rightSpeed(0){}

ArgosRosBridge::~ArgosRosBridge(){}

void ArgosRosBridge::Init(TConfigurationNode& t_node){
	/********************************
	 * For the robot sensors:
	 * 1. Get sensor handles
	 * 2. Create the topics to publish
	 ********************************/
	if (HasSensor("footbot_light")){
		stringstream lightTopic;
		lightTopic 			<< "/" << GetId() << "/lightList";
		m_pcLight  			= GetSensor < CCI_FootBotLightSensor>("footbot_light");
		lightListPublisher_ = ArgosRosBridge::nodeHandle -> create_publisher<LightList>(lightTopic.str(), 1);

	}
	if (HasSensor("footbot_proximity")){
		stringstream proxTopic;
		proxTopic 			<< "/" << GetId() << "/proximityList";
		m_pcProximity 		= GetSensor < CCI_FootBotProximitySensor>("footbot_proximity");
		promixityListPublisher_ = ArgosRosBridge::nodeHandle -> create_publisher<ProximityList>(proxTopic.str(), 1);
	}
	if (HasSensor("positioning")){
		stringstream positionTopic;
		positionTopic 		<< "/" << GetId() << "/position";
		m_pcPosition 		= GetSensor < CCI_PositioningSensor>("positioning");
		positionPublisher_ 	= ArgosRosBridge::nodeHandle -> create_publisher<Position>(positionTopic.str(), 1);
	}
	if (HasSensor("range_and_bearing")){
		stringstream rabTopic;
		rabTopic 			<< "/" << GetId() << "/rab";
		m_pcRABS 			= GetSensor < CCI_RangeAndBearingSensor>("range_and_bearing");
		rabPublisher_ 		= ArgosRosBridge::nodeHandle -> create_publisher<PacketList>(rabTopic.str(), 1);
	}
	if (HasSensor("colored_blob_omnidirectional_camera")){
		stringstream blobTopic;
		blobTopic 			<< "/" << GetId() << "/blobList";
		m_pcCamera 			= GetSensor < CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
		blobListPublisher_ 	= ArgosRosBridge::nodeHandle -> create_publisher<BlobList>(blobTopic.str(), 1);
	}

	/********************************
	 * For the robot actuators:
	 * 1. Get actuator handles
	 * 2. Create the subscribers to subscribe to the topics
	 ********************************/
	if (HasActuator("leds")){
		m_pcLEDs = GetActuator< CCI_LEDsActuator >("leds");
		stringstream cmdLedTopic;
		cmdLedTopic 	<< "/" << GetId() << "/cmd_led";
		cmdLedSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Led>(
							cmdLedTopic.str(),
							1,
							std::bind(&ArgosRosBridge::cmdLedCallback, this, _1)
							);
	}
	if (HasActuator("range_and_bearing")){
		m_pcRABA = GetActuator< CCI_RangeAndBearingActuator >("range_and_bearing");
		stringstream cmdRabTopic;
		cmdRabTopic 	<< "/" << GetId() << "/cmd_rab";
		cmdRabSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Packet>(
							cmdRabTopic.str(),
							1,
							std::bind(&ArgosRosBridge::cmdRabCallback, this, _1)
							);
	}

	if (HasActuator("differential_steering")){
		m_pcWheels = GetActuator< CCI_DifferentialSteeringActuator >("differential_steering");
		stringstream cmdVelTopic;
		cmdVelTopic 	<< "/" << GetId() << "/cmd_vel";
		cmdVelSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Twist>(
							cmdVelTopic.str(),
							1,
							std::bind(&ArgosRosBridge::cmdVelCallback, this, _1)
							);
	}

	/*
	* Other init stuff
	*/
	if (HasSensor("colored_blob_omnidirectional_camera")){
	/* Enable camera filtering */
	   m_pcCamera->Enable();
	}
	/*
	* Parse the configuration file
	*
	* The user defines this part. Here, the algorithm accepts three
	* parameters and it's nice to put them in the config file so we don't
	* have to recompile if we want to try other settings.
	*/
	GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);

}

bool blobComparator(Blob a, Blob b) {
	return a.angle < b.angle;
}

void ArgosRosBridge::ControlStep() {

	rclcpp::spin_some(ArgosRosBridge::nodeHandle);

	/*********************************
	 * Get readings from light sensor
	 *********************************/
	if (HasSensor("footbot_light")){
		const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
		LightList lightList;
		lightList.n = tLightReads.size();
		for (size_t i = 0; i < lightList.n; ++i) {
			Light light;
			light.value = tLightReads[i].Value;
			light.angle = tLightReads[i].Angle.GetValue();
			lightList.lights.push_back(light);

		}

		lightListPublisher_ -> publish(lightList);
	}
	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
	if (HasSensor("footbot_proximity")){
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
	if (HasSensor("colored_blob_omnidirectional_camera")){
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcCamera->GetReadings();
		BlobList blobList;
		blobList.n = camReads.BlobList.size();
		Blob blob;
		for (size_t i = 0; i < blobList.n; ++i) {
			//Blob blob;
			stringstream ss;
			ss << camReads.BlobList[i]->Color;
			blob.color = ss.str();
			blob.distance = camReads.BlobList[i]->Distance;

			// Make the angle of the puck in the range [-PI, PI].  This is useful for
			// tasks such as homing in on a puck using a simple controller based on
			// the sign of this angle.
			blob.angle = camReads.BlobList[i]->Angle.GetValue();//.SignedNormalize().GetValue();
			blobList.blobs.push_back(blob);

		}

		// Sort the blob list by angle.  This is useful for the purposes of extracting meaning from
		// the local blob configuration (e.g. fitting a lines to the detected blobs).
		sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

		blobListPublisher_ -> publish(blobList);
	}

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
	if (HasSensor("positioning")){
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
	}

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	if (HasSensor("range_and_bearing")){
		const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
		PacketList packetList;
		packetList.n = tRabReads.size();
		for (size_t i = 0; i < packetList.n; ++i) {
			Packet packet;
			packet.range = tRabReads[i].Range;
			packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
			packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

			
			packet.data.push_back(tRabReads[i].Data[0]);
			packet.data.push_back(tRabReads[i].Data[1]);

			packetList.packets.push_back(packet);
			
		}

		rabPublisher_ -> publish(packetList);
	}

	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount) {
		leftSpeed = 0;
		rightSpeed = 0;
	} else {
		stepsSinceCallback++;
	}
	if (HasActuator("differential_steering")){
		m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
	}
}

void ArgosRosBridge::cmdVelCallback(const Twist& twist) {
	double v = twist.linear.x;		// Forward linear velocity
	double omega = twist.angular.z; // Rotational (angular) velocity
	double L = HALF_BASELINE * 2;	// Distance between wheels (wheelbase)
	double R = WHEEL_RADIUS;		// Wheel radius

	// Calculate left and right wheel speeds using differential drive kinematics
	//leftSpeed = (v - (L / 2) * omega) / R;
	//rightSpeed = (v + (L / 2) * omega) / R;

	leftSpeed = twist.linear.x;
	rightSpeed = twist.linear.y;

	stepsSinceCallback = 0;
}

void ArgosRosBridge::cmdRabCallback(const Packet& packet){
	//cout << GetId() << " Packet data as received: " << packet.data[0] << " for id: " << std::stoi( packet.id ) <<endl;
	m_pcRABA -> SetData(0, packet.data[0]);
	m_pcRABA -> SetData(1, std::stoi( packet.id ));
}
void ArgosRosBridge::cmdLedCallback(const Led& ledColor){
	/**
	 * TODO: Btter way to set the led colors	
	 */
	if ( ledColor.color == "red" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::RED);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::RED);
		}
	}
	 else if ( ledColor.color == "yellow" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::YELLOW);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::YELLOW);
		}
	}
	else if ( ledColor.color == "green" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::GREEN);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::GREEN);
		}
	}
	else if ( ledColor.color == "magenta" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::MAGENTA);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::MAGENTA);
		}
	}
	else if ( ledColor.color == "black"){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::BLACK);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::BLACK);
		}
	}
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
