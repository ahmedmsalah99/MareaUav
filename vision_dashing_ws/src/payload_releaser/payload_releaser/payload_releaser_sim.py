import rclpy

import os

import sys

from rclpy.node import Node

from std_msgs.msg import String

#import RPi.GPIO as GPIO
import time



class PayloadReleaser(Node):
	payloadReleased = False

	def __init__(self):
		super().__init__('payload_releaser')
		self.subscription = self.create_subscription(String, 'release_payload', self.listener_callback, 1)
		self.subscription # prevent unused variable warning

		self.publisher = self.create_publisher(String, "release_payload_feedback", 1)
		self.payloadReleased = False

	def listener_callback(self, msg):
		#self.get_logger().info('I heard: "%s"' % msg.data)
		if msg.data == 'T':
			if self.payloadReleased == False:
				print ("Received payload release cmd.")
				# GPIO 24 is the one that controls the PWM signal given to the servo
				#servoPIN = 24
				#GPIO.setmode(GPIO.BCM)
				#GPIO.setup(servoPIN, GPIO.OUT)

				#p = GPIO.PWM(servoPIN, 50) # 50Hz PWM

				#p.start(2.5) # Init
				try:
					print ("Arm")
					#p.ChangeDutyCycle(0)
					time.sleep(1.5)
					print ("Opening")
					#p.ChangeDutyCycle(0.08)
					time.sleep(4)
					msg1 = String()
					msg1.data = "W"
					self.publisher.publish(msg1)
					time.sleep(1)
					print ("Disarm")
					#p.stop()
					#GPIO.cleanup()
					self.payloadReleased = True

					msg2 = String()
					msg2.data = "R"
					self.publisher.publish(msg2)

				except KeyboardInterrupt:
					#p.stop()
					#GPIO.cleanup()
					pass

			else:
				print ("Payload already released.")
		else:
			print ("Received unknown command.")

def main(args=None):
	rclpy.init(args=args)

	payload_releaser = PayloadReleaser()
	print("Payload releaser waiting for command")

	try:
		while True:
			rclpy.spin(payload_releaser)
	except KeyboardInterrupt:
	    pass


	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	payload_releaser.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
