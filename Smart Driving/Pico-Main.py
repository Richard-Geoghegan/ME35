# Pico-Main.py
# Richard Geoghegan (github.com/Richard-Geoghegan
# main.py for Maker Pico

import time
from mqtt import MQTTClient
from wifi import *
from machine import Pin, PWM

connect_wifi()  # connect to wifi using  custom wifi module
mqtt_broker = 'broker.hivemq.com'
port = 1883
topic_sub = 'ME35-24/noahcam'
desired_x = 0  # Desired position, 0 is center
desired_z = -10
x_pos = desired_x
z_pos = desired_z
found_tag = True

def callback(topic, msg):
    # callback for new mqtt message on topic
    global x_pos, z_pos, found_tag
    # decode the message and update position variables
    string = msg.decode()
    if string[1] == ",":       # confirm correct format coming from camera
        tag, x, z = string.split(',')
        if int(tag) == 1:
            found_tag = True
            x_pos = float(x)
            z_pos = float(z)
        else:
            found_tag = False
            x_pos = desired_x
            z_pos = desired_z
          
client = MQTTClient('motorcontrol', mqtt_broker , port)
client.connect()
print('Connected to %s MQTT broker' % (mqtt_broker))
client.set_callback(callback)          # set the callback if anything is read
client.subscribe(topic_sub.encode())   # subscribe to a bunch of topics

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
kp = 8.0  # Proportional gain
kd = 1.0  # Derivative gain
dead_zone = 6000  # dead zone threshold, tune for motor, 0 does nothing

# Variables to track previous error for the derivative term
previous_error = 0
previous_time = time.ticks_ms()

while True:
    client.check_msg() # check for new messages
    if found_tag:
        error = z_pos - desired_z  # Calculate error (difference from the center)
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, previous_time) / 1000.0  # Convert to seconds
      
        # Calculate derivative of the error (rate of change)
        if delta_time > 0:
            derivative = (error - previous_error) / delta_time
        else:
            derivative = 0
          
        # PD control signal
        control_signal = (kp * error) + (kd * derivative)
        pwm_val = abs(control_signal*1000) + dead_zone
        pwm_val = min(65535, int(pwm_val))

        # Update previous error and time
        previous_error = error
        previous_time = current_time
        print(z_pos, error, pwm_val)
      
        if control_signal > 0:
            pwm2.duty_u16(0)  # Ensure backward pin is off
            pwm3.duty_u16(pwm_val)  # Apply PWM to m1
            pwm4.duty_u16(0)  # Ensure backward pin is off
            pwm5.duty_u16(pwm_val)  # Apply PWM to m1
        else:
            pwm3.duty_u16(0)  # Ensure forward pin is off
            pwm2.duty_u16(pwm_val)  # Apply PWM to m2
            pwm5.duty_u16(0)  # Ensure forward pin is off
            pwm4.duty_u16(pwm_val)  # Apply PWM to m2
    else:
        pwm2.duty_u16(0)
        pwm3.duty_u16(0)
        pwm5.duty_u16(0)  # Ensure forward pin is off
        pwm4.duty_u16(0)  # Apply PWM to m2
    time.sleep_ms(10)
