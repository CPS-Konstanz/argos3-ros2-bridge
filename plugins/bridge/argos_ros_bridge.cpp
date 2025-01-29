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
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabTopic;
	lightTopic 		<< "/" << GetId() << "/light";
	blobTopic 		<< "/" << GetId() << "/blob";
	proxTopic 		<< "/" << GetId() << "/proximity";
	positionTopic 	<< "/" << GetId() << "/position";
	rabTopic 		<< "/" << GetId() << "/rab";
	lightListPublisher_ = ArgosRosBridge::nodeHandle -> create_publisher<LightList>(lightTopic.str(), 1);
	blobPublisher_ 		= ArgosRosBridge::nodeHandle -> create_publisher<BlobList>(blobTopic.str(), 1);
	promixityPublisher_ = ArgosRosBridge::nodeHandle -> create_publisher<ProximityList>(proxTopic.str(), 1);
	positionPublisher_ 	= ArgosRosBridge::nodeHandle -> create_publisher<Position>(positionTopic.str(), 1);
	rabPublisher_ 		= ArgosRosBridge::nodeHandle -> create_publisher<PacketList>(rabTopic.str(), 1);

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic 	<< "/" << GetId() << "/cmd_vel";
	cmdRabTopic		<< "/" << GetId() << "/cmd_rab";
	cmdLedTopic		<< "/" << GetId() << "/cmd_led";
	cmdVelSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Twist>(
						cmdVelTopic.str(),
						1,
						std::bind(&ArgosRosBridge::cmdVelCallback, this, _1)
						);
	cmdRabSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Packet>(
						cmdRabTopic.str(),
						1,
						std::bind(&ArgosRosBridge::cmdRabCallback, this, _1)
						);
	cmdLedSubscriber_ = ArgosRosBridge::nodeHandle -> create_subscription<Led>(
						cmdLedTopic.str(),
						1,
						std::bind(&ArgosRosBridge::cmdLedCallback, this, _1)
						);
	/********************************
	 * Get sensor handles
	 ********************************/
	m_pcLight  		= GetSensor < CCI_FootBotLightSensor					>("footbot_light");
	m_pcProximity 	= GetSensor < CCI_FootBotProximitySensor				>("footbot_proximity");
	m_pcLEDs   		= GetActuator<CCI_LEDsActuator                          >("leds");
	m_pcCamera 		= GetSensor < CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	m_pcPosition 	= GetSensor < CCI_PositioningSensor						>("positioning");
	m_pcRABS 		= GetSensor < CCI_RangeAndBearingSensor					>("range_and_bearing" );

	/********************************
	 * Get actuator handles
	 ********************************/
	m_pcWheels = GetActuator< CCI_DifferentialSteeringActuator >("differential_steering");
	m_pcRABA = GetActuator< CCI_RangeAndBearingActuator        >("range_and_bearing" );

	/*
	* Other init stuff
	*/
	Reset();
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

	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	ProximityList proxList;
	proxList.n = tProxReads.size();
	for (size_t i = 0; i < proxList.n; ++i) {
		Proximity prox;
		prox.value = tProxReads[i].Value;
		prox.angle = tProxReads[i].Angle.GetValue();
		proxList.proximities.push_back(prox);

	}

	promixityPublisher_ -> publish(proxList);

	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
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

	// Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
	// the local puck configuration (e.g. fitting a lines to the detected pucks).
	sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

	blobPublisher_ -> publish(blobList);

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
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

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
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

	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount) {
		leftSpeed = 0;
		rightSpeed = 0;
	} else {
		stepsSinceCallback++;
	}
	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void ArgosRosBridge::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /** Set the LED intensity first */
   m_pcLEDs -> SetAllIntensities( 100 );
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetAllColors(CColor::RED);
}

void ArgosRosBridge::cmdVelCallback(const Twist& twist) {
	double v = twist.linear.x;		// Forward linear velocity
	double omega = twist.angular.z; // Rotational (angular) velocity
	double L = HALF_BASELINE * 2;	// Distance between wheels (wheelbase)
	double R = WHEEL_RADIUS;		// Wheel radius

	// Calculate left and right wheel speeds using differential drive kinematics
	leftSpeed = (v - (L / 2) * omega) / R;
	rightSpeed = (v + (L / 2) * omega) / R;

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
	if ( ledColor.color == "yellow" ){
		m_pcLEDs->SetAllColors(CColor::YELLOW);
	}
	else if ( ledColor.color == "green" ){
		m_pcLEDs->SetAllColors(CColor::GREEN);
	}
	else if ( ledColor.color == "magenta" ){
		m_pcLEDs->SetAllColors(CColor::MAGENTA);
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
