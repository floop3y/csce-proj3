#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose
from duckietown.dtros import DTROS, NodeType, TopicType

class PIDController:
	def __init__(self, P, I, D):
		self.P = P
		self.I = I
		self.D = D
		self.prev_error = 0
		self.error_sum = 0

	def update(self, error, time):
		self.error_sum += error*time
		self.error_sum = self.checkBounds(self.error_sum, -1.2, 1.2)
		error = self.checkBounds(error, -0.75,0.8)
		deriv = (error-self.prev_error)/time
		output = self.P*error + self.I*self.error_sum + self.D*deriv
		self.prev_error = error
		output = self.checkBounds(output, -3.75,3.75)
		return output

	def checkBounds(self, value, lower, upper):
		if value < lower:
			return lower;
		elif value > upper:
			return upper;
		else:
			return value;

class LaneControllerNode(DTROS):
	def __init__(self):
		super(LaneControllerNode, self).__init__(node_name="Proj3", node_type=NodeType.PERCEPTION)
		self.pose=LanePose()
		self.pub_car_cmd = rospy.Publisher("duckie/lane_controller_node/car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
		self.sub_lane = rospy.Subscriber("duckie/lane_filter_node/lane_pose", LanePose, self.lane_pose_callback, queue_size = 1)
		self.D_pid = PIDController(P=9, I=0.04, D=0.3)
		self.phi_pid = PIDController(P=6, I=0.02, D=0.25)
		self.last_s=1
		self.current_lane_pose=None

	def lane_pose_callback(self, msg):
		self.pose = msg
		rospy.logwarn("Init Lane Following Code")
		self.getCommand()

	def getCommand(self):
		current_s = rospy.Time.now().to_sec()
		dt = 1
		if self.last_s is not None:
			dt = current_s - self.last_s
			d_err = self.pose.phi
			omega = self.D_pid.update(d_err,dt) + self.phi_pid.update(phi_err, dt)
			omega = omega * -1
			car_control_msg = Twisted2DStamped()
			car_control_msg.header = self.pose.header
			car_control_msg.v = 0.25
			car_control_msg.omega = omega
			self.pub_car_cmd.publish(car_control_msg)
			self.last_s = current_s

if __name__ == "__main__":
	lane_controller_node = LaneControllerNode()
	rospy.spin()
