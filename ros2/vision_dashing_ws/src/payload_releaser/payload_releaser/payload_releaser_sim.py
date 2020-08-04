import rclpy

import lcm

import os

m2m_include = os.environ['M2M_INCLUDE']

import sys

sys.path.insert(1, m2m_include)

from lcm_messages.exec import release_payload

from rclpy.node import Node

from std_msgs.msg import String

#import RPi.GPIO as GPIO
import time



class PayloadReleaser(Node):
	payloadReleased = False

	def __init__(self):
		super().__init__('payload_releaser')
		self.payloadReleased = False

	def listener_callback(self, channel, data):
		#self.get_logger().info('I heard: "%s"' % msg.data)
		msg = release_payload.decode(data)
		if msg.data == 'T':
			print ("Received payload release cmd.")
			if self.payloadReleased == False:
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
					time.sleep(5)
					print ("Disarm")
					#p.stop()
					#GPIO.cleanup()
					self.payloadReleased = True

					msg = release_payload()
					msg.data = "R"
					lc = lcm.LCM()
					lc.publish("release_payload_feedback", msg.encode())

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

	lc=lcm.LCM()
	subscription = lc.subscribe('release_payload', payload_releaser.listener_callback)
	subscription  # prevent unused variable warning

	try:
		while True:
			lc.handle()
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
