"""The purpose of this script is to mimic real-world noisy sensor output"""

import rospy
import random as rand
from nav_msgs.msg import Odometry

class NoisyOdom():
	def __init__(self):
		# Subscribe to the odometry topic 
		self.odometry_subscriber = rospy.Subscriber('/bowser2/odom', Odometry, self.odom_callback)
		# Publish a noisy odometry topic
		self.noisy_odom_publisher = rospy.Publisher('/noisy_odom', Odometry, queue_size = 1)
		# Create an odometry message
		self.odometry_msg = Odometry()
		# Set the publisher's loop frequency
		self.rate = rospy.Rate(5) # 5Hz

	def odom_callback(self, msg):
		# Save the odometry message from /bowser2/odom
		self.odometry_msg = msg
		# Call the add noise function
		self.add_noise()

	def add_noise(self):
		# Add noise to the X and Y position value of the odometry message
		rand_float = rand.uniform(-0.5, 0.5) 
		self.odometry_msg.pose.pose.position.x = self.odometry_msg.pose.pose.position.x + rand_float
		self.odometry_msg.pose.pose.position.x = self.odometry_msg.pose.pose.position.y + rand_float

	def publish_noisy_odom(self):
		while not rospy.is_shutdown():
			self.noisy_odom_publisher.publish(self.odometry_msg) # Publish noisy odom message to /noisy_odom topic
			self.rate.sleep() # Sleep the required time to meet loop frequency set previsously

if __name__ == "__main__":
	# Create a node
	rospy.init_node("noisy_odometry_node")
	noisy_odom = NoisyOdom()
	try:
		noisy_odom.publish_noisy_odom()
	except rospy.ROSInterruptException:
		pass
