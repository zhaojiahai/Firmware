#include "ros.hpp"
#include "px4_posix.h"
#include "drivers/drv_hrt.h"

namespace ros
{

Time::Time() : _nsec(0)
{
}

Time::Time(uint64_t nsec) : _nsec(nsec)
{
}

Time Time::fromNSec(uint64_t nsec)
{
	return Time(nsec);
}

uint64_t Time::toNSec()
{
	return _nsec;
}

Time Time::now()
{
	return fromNSec(1e3 * hrt_absolute_time());
}

// The node for this process
Node *_node = NULL;

void init(int argc, char **argv, const char *node_name)
{
	_node = new Node();
}

void spin()
{
	if (_node != NULL) {
		//ROS_INFO("node spinning");
		_node->spin();

	} else {
		ROS_INFO("node not initialized");
	}
}

Node::Node() :
	_subListHead(NULL),
	_hmap()
{
	_hmap.put("vehicle_attitude", ORB_ID(vehicle_attitude));
	_hmap.put("sensor_combined", ORB_ID(sensor_combined));
	_hmap.put("vision_position_estimate", ORB_ID(vision_position_estimate));
	_hmap.put("att_pos_mocap", ORB_ID(att_pos_mocap));
	_hmap.put("airspeed", ORB_ID(airspeed));
	_hmap.put("parameter_update", ORB_ID(parameter_update));
	_hmap.put("vehicle_global_position", ORB_ID(vehicle_global_position));
	_hmap.put("vehicle_local_position", ORB_ID(vehicle_local_position));
	_hmap.put("vehicle_gps_position", ORB_ID(vehicle_gps_position));
	_hmap.put("sensor_accel", ORB_ID(sensor_accel));
	_hmap.put("sensor_gyro", ORB_ID(sensor_gyro));
	_hmap.put("sensor_mag", ORB_ID(sensor_mag));
	_hmap.put("sensor_baro", ORB_ID(sensor_baro));
	_hmap.put("control_state", ORB_ID(control_state));
	_hmap.put("estimator_status", ORB_ID(estimator_status));
	_hmap.put("airspeed", ORB_ID(airspeed));
}

void Node::spin()
{
	Subscriber *tail = _subListHead;

	while (tail != NULL) {
		tail->callback();
		tail = tail->next;
	}
}

void Node::addSubscriber(Subscriber *sub)
{
	if (_subListHead == NULL) {
		_subListHead = sub;
		return;
	}

	Subscriber *tail = _subListHead;

	while (tail->next != NULL) {
		tail = tail->next;
	}

	tail->next = sub;
}

bool Node::getTopicMeta(const char *topic, const struct orb_metadata **meta)
{
	return _hmap.get(topic, *meta);
}


Rate::Rate(float frequency):
	_frequency(frequency),
	_wake_timestamp(hrt_absolute_time())
{
}

void Rate::sleep()
{
	int32_t dt = _wake_timestamp - hrt_absolute_time();

	if (dt > 0) {
		usleep(dt);
	}

	_wake_timestamp = hrt_absolute_time() + 1.0e6f / _frequency;
}

Subscriber::Subscriber() :
	next(NULL),
	_callbackPtr(NULL)
{
}

Subscriber::Subscriber(CallbackInterface *cb) :
	next(NULL),
	_callbackPtr(cb)
{
	if (_node != NULL) {
		_node->addSubscriber(this);

	} else {
		ROS_INFO("node not initialized");
	}
}


Subscriber::~Subscriber()
{
}

void Subscriber::callback()
{
	if (_callbackPtr != NULL) {
		_callbackPtr->callback();

	} else {
		ROS_INFO("callback ptr is NULL!");
	}
}

Publisher::Publisher(uORB::PublicationTiny *pub) :
	_pub(pub)
{
}

Publisher::~Publisher()
{
}

bool NodeHandle::ok()
{
	return true;
}

void NodeHandle::param(const char *name, float val, const char *topic)
{
}

}
