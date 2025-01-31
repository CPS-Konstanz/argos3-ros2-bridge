/*
 * argos_ros_bridge.h
 *
 * Created on: 28 Jan 2025
 *  
 */

#ifndef ARGOS_ROS_BRIDGE_H_
#define ARGOS_ROS_BRIDGE_H_

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include <cuchar>

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "argos3_ros2_bridge/msg/led.hpp"
#include "argos3_ros2_bridge/msg/light_list.hpp"
#include "argos3_ros2_bridge/msg/blob_list.hpp"
#include "argos3_ros2_bridge/msg/proximity_list.hpp"
#include "argos3_ros2_bridge/msg/position.hpp"
#include "argos3_ros2_bridge/msg/packet.hpp"
#include "argos3_ros2_bridge/msg/packet_list.hpp"

using namespace argos;
using namespace std::chrono_literals;
class ArgosRosBridge : public CCI_Controller{
    private:
		/************************************
		 * Publishers
		 ***********************************/
		// Light list publisher
		rclcpp::Publisher<argos3_ros2_bridge::msg::LightList>::SharedPtr lightListPublisher_;
		// Proximity sensor publisher
		rclcpp::Publisher<argos3_ros2_bridge::msg::ProximityList>::SharedPtr promixityListPublisher_;
		// Omnidirectional camera sensor publisher
		rclcpp::Publisher<argos3_ros2_bridge::msg::BlobList>::SharedPtr blobListPublisher_;
		// Position sensor publisher
		rclcpp::Publisher<argos3_ros2_bridge::msg::Position>::SharedPtr positionPublisher_;
		// Position sensor publisher
		rclcpp::Publisher<argos3_ros2_bridge::msg::PacketList>::SharedPtr rabPublisher_;

		/************************************
		 * Subscribers
		 ***********************************/
		// Subscriber for cmd_vel (Twist message) topic.
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
		// Subscriber for cmd_rab (Packet) topic
		rclcpp::Subscription<argos3_ros2_bridge::msg::Packet>::SharedPtr cmdRabSubscriber_;
		// Subscriber for cmd_led (Led) topic
		rclcpp::Subscription<argos3_ros2_bridge::msg::Led>::SharedPtr cmdLedSubscriber_;



		rclcpp::TimerBase::SharedPtr timer_;


		/* Pointer to the differential steering actuator */
		CCI_DifferentialSteeringActuator* m_pcWheels;
		/* Pointer to the foot-bot light sensor */
		CCI_FootBotLightSensor* m_pcLight;
		/* Pointer to the LEDs actuator */
		CCI_LEDsActuator* m_pcLEDs;
		/* Pointer to the omnidirectional camera sensor */
		CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
		/* Pointer to proximity sensor */
		CCI_FootBotProximitySensor* m_pcProximity;
		/* Pointer to positioning sensor */
		CCI_PositioningSensor* m_pcPosition;
		/* Pointer to the range-and-bearing sensor */
		CCI_RangeAndBearingSensor* m_pcRABS;
		/* Pointer to the range-and-bearing actuator */
		CCI_RangeAndBearingActuator* m_pcRABA;

		// The following constant values were copied from the argos source tree from
		// the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
		static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
		static constexpr Real WHEEL_RADIUS = 0.029112741f;

		/*
		* The following variables are used as parameters for the
		* algorithm. You can set their value in the <parameters> section
		* of the XML configuration file, under the
		* <controllers><argos_ros_bot_controller> section.
		*/

		// The number of time steps from the time step of the last callback
		// after which leftSpeed and rightSpeed will be set to zero.  Useful to
		// shutdown the robot after the controlling code on the ROS side has quit.
		int stopWithoutSubscriberCount;

		// The number of time steps since the last callback.
		int stepsSinceCallback;

		// Most recent left and right wheel speeds, converted from the ROS twist
		// message.
		Real leftSpeed, rightSpeed;


    public:

		ArgosRosBridge();

		virtual ~ArgosRosBridge();

		/*
		* This function initializes the controller.
		* The 't_node' variable points to the <parameters> section in the XML
		* file in the <controllers><footbot_ccw_wander_controller> section.
		*/
		virtual void Init(TConfigurationNode& t_node);

		/*
		* This function is called once every time step.
		* The length of the time step is set in the XML file.
		*/
		virtual void ControlStep();
		/*
		* This function resets the controller to its state right after the
		* Init().
		* It is called when you press the reset button in the GUI.
		* In this example controller there is no need for resetting anything,
		* so the function could have been omitted. It's here just for
		* completeness.
		*/
		virtual void Reset();
		/*
		* Called to cleanup what done by Init() when the experiment finishes.
		*/
		virtual void Destroy() {}
		/*
		* The callback method for getting new commanded speed on the cmd_vel topic.
		*/
		void cmdVelCallback(const geometry_msgs::msg::Twist& twist);
		/*
		 * The callback method for getting new commanded packet on the cmd_packet topic.
		 */
		void cmdRabCallback(const argos3_ros2_bridge::msg::Packet& packet);
		/*
		 * The callback method for getting new commanded led color on the cmd_led topic.
		 */
		void cmdLedCallback(const argos3_ros2_bridge::msg::Led& ledColor);

		static std::shared_ptr<rclcpp::Node> nodeHandle;
};
#endif /* ARGOS_ROS_BRIDGE_H_ */