import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def clamp(value, limit):
	if value > limit:
		return limit
	if value < -limit:
		return -limit
	return value
    

class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = True
		self.linear_Gear = 1
		self.angular_Gear = 1
		self._previous_buttons = []
		
		#create pub
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)
		
		#declare parameter and get the value
		self.declare_parameter('xspeed_limit',0.5)
		self.declare_parameter('yspeed_limit',0.5)
		self.declare_parameter('angular_speed_limit',1.0)
		self.declare_parameter('axis_linear_x',1)
		self.declare_parameter('axis_linear_y',0)
		self.declare_parameter('axis_angular',3)
		self.declare_parameter('linear_gear_button',9)
		self.declare_parameter('angular_gear_button',10)
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		self.axis_linear_x = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
		self.axis_linear_y = self.get_parameter('axis_linear_y').get_parameter_value().integer_value
		self.axis_angular = self.get_parameter('axis_angular').get_parameter_value().integer_value
		self.linear_gear_button = self.get_parameter('linear_gear_button').get_parameter_value().integer_value
		self.angular_gear_button = self.get_parameter('angular_gear_button').get_parameter_value().integer_value

	def buttonCallback(self,joy_data):
		if not isinstance(joy_data, Joy):
			return

		if self.button_pressed(joy_data, self.linear_gear_button):
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1

		if self.button_pressed(joy_data, self.angular_gear_button):
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0

		xlinear_speed = self.filter_data(self.axis_value(joy_data, self.axis_linear_x)) * self.xspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(self.axis_value(joy_data, self.axis_linear_y)) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(self.axis_value(joy_data, self.axis_angular)) * self.angular_speed_limit * self.angular_Gear

		twist = Twist()
		twist.linear.x = clamp(xlinear_speed, self.xspeed_limit)
		twist.linear.y = clamp(ylinear_speed, self.yspeed_limit)
		twist.angular.z = clamp(angular_speed, self.angular_speed_limit)
		if self.Joy_active:
			self.pub_cmdVel.publish(twist)
			self.pub_JoyState.publish(Bool(data=True))
		self._previous_buttons = list(joy_data.buttons)

	def axis_value(self, joy_data, axis_index):
		if axis_index < 0 or axis_index >= len(joy_data.axes):
			return 0.0
		return joy_data.axes[axis_index]

	def button_pressed(self, joy_data, button_index):
		if button_index < 0 or button_index >= len(joy_data.buttons):
			return False
		was_pressed = (
			button_index < len(self._previous_buttons)
			and self._previous_buttons[button_index] == 1
		)
		return joy_data.buttons[button_index] == 1 and not was_pressed
        
	def filter_data(self, value):
		if abs(value) < 0.2: value = 0
		return value		
			
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	try:
		rclpy.spin(joy_ctrl)
	finally:
		joy_ctrl.destroy_node()
		rclpy.shutdown()
	
if __name__ == '__main__':
	main()
