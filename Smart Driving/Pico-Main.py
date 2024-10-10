# Pico-Main.py
# Richard Geoghegan (github.com/Richard-Geoghegan
# main.py for Maker Pico
# Drives both motors on Pico based on X, Z position of the AprilTag

import time
from mqtt import MQTTClient
from wifi import *
from machine import Pin, PWM

connect_wifi()  # connect to wifi using custom wifi module

mqtt_broker = 'broker.hivemq.com'
port = 1883
topic_sub = 'ME35-24/RichardCam'

desired_x = 0  # Desired position, 0 is center
desired_z = -7

x_pos = desired_x
z_pos = desired_z
found_tag = True

def callback(topic, msg):
    global x_pos, z_pos, found_tag
    string = msg.decode()
    if string[1] == ",":  # confirm correct format from camera
        tag, x, z = string.split(',')
        if int(tag) == 1:
            found_tag = True
            x_pos = float(x)
            z_pos = float(z)
        else:
            found_tag = False
            x_pos = desired_x
            z_pos = desired_z

client = MQTTClient('motorcontrol', mqtt_broker, port)
client.connect()
print('Connected to %s MQTT broker' % (mqtt_broker))
client.set_callback(callback)  # set the callback for messages
client.subscribe(topic_sub.encode())  # subscribe to the topic

# Setup PWM control for four pins, two for each motor
pwm2 = PWM(Pin(2))
pwm3 = PWM(Pin(3))

pwm4 = PWM(Pin(4))
pwm5 = PWM(Pin(5))

pwm2.freq(1000)
pwm3.freq(1000)

pwm4.freq(1000)
pwm5.freq(1000)

# PD controller gains
kp_speed = 7.0  # Proportional gain for speed
kd_speed = 1.0  # Derivative gain for speed

kp_turn = 2.0  # Proportional gain for turning
kd_turn = 0.5  # Derivative gain for turning

dead_zone = 6000  # dead zone threshold, tune for motor

# Variables to track previous error for PD controllers
previous_error_speed = 0
previous_error_turn = 0
previous_time = time.ticks_ms()

# PD controller for speed
def pd_controller_speed(error, previous_error, delta_time):
    if delta_time > 0:
        derivative = (error - previous_error) / delta_time
    else:
        derivative = 0
    control_signal = (kp_speed * error) + (kd_speed * derivative)
    return control_signal * 1000

# PD controller for turning
def pd_controller_turn(x_pos, previous_x_pos, delta_time):
    error_turn = x_pos - desired_x
    if delta_time > 0:
        derivative_turn = (x_pos - previous_x_pos) / delta_time
    else:
        derivative_turn = 0
    turn_signal = (kp_turn * error_turn) + (kd_turn * derivative_turn)
    return turn_signal * 1000.0

# Function to control motors based on speed and turn signals
def control_motors(control_signal_speed, turn_signal):
    # Calculate PWM for speed
    pwm_val = abs(control_signal_speed) + dead_zone
    pwm_val = min(65535, int(pwm_val))
    turn_signal = int(turn_signal)

    # Adjust motor PWM values based on turn signal
    if control_signal_speed > 0:  # Moving forward
        pwm2.duty_u16(0)   # Ensure backward pin is off
        pwm3.duty_u16(pwm_val - turn_signal)  # Apply PWM to right motor
            
        pwm4.duty_u16(0)   # Ensure backward pin is off
        pwm5.duty_u16(pwm_val + turn_signal)  # Reduce left motor speed
    else:  # Moving backward
        pwm3.duty_u16(0)  # Ensure forward pin is off
        pwm2.duty_u16(pwm_val + turn_signal)  # Reverse left motor
        
        pwm5.duty_u16(0)  # Ensure forward pin is off
        pwm4.duty_u16(pwm_val - turn_signal)  # Reverse right motor

while True:
    client.check_msg()  # check for new messages

    if found_tag:
        # Time tracking
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, previous_time) / 1000.0  # Convert to seconds

        # Speed control based on z_pos
        error_speed = z_pos - desired_z
        control_signal_speed = pd_controller_speed(error_speed, previous_error_speed, delta_time)

        # Turn control based on x_pos
        turn_signal = pd_controller_turn(x_pos, previous_error_turn, delta_time)

        # Update previous errors and time
        previous_error_speed = error_speed
        previous_error_turn = x_pos
        previous_time = current_time
        
        print(control_signal_speed, turn_signal)

        # Control the motors using the control signals
        control_motors(control_signal_speed, turn_signal)
    else:
        pwm2.duty_u16(0)
        pwm3.duty_u16(0)
        pwm5.duty_u16(0)
        pwm4.duty_u16(0)

    time.sleep_ms(10)
