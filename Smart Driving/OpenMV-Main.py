# OpenMV-Main.py
# Richard Geoghegan
# main.py for OpenMV camera for the smart driving project
# Sends current X, Z position and size of AprilTag in view over MQTT

import time
import network
from mqtt import MQTTClient
from secrets import mysecrets
import machine
import sensor

SSID = mysecrets['SSID']  # Network SSID
KEY = mysecrets['key']    # Network Password

mqtt_broker = 'broker.hivemq.com'

topic_pub = 'ME35-24/noahcam'

# Init wlan module and connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, KEY)

while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(SSID))
    time.sleep_ms(1000)

# We should have a valid IP now via DHCP
print("WiFi Connected ", wlan.ifconfig())

client = MQTTClient("openmv_noah", mqtt_broker, port=1883)
client.connect()

# Setup camera
sensor.reset()

# Set to grayscale and QQVGA for faster framerate
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...

f_x = (2.8 / 3.984) * 160  # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120  # find_apriltags defaults to this if not set
c_x = 160 * 0.5  # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags defaults to this if not set (the image.h * 0.5)

found_tag = 0
while True:
    # get x, y coordinates of a tag
    x, z = None, None
    img = sensor.snapshot()
    for tag in img.find_apriltags(
        fx=f_x, fy=f_y, cx=c_x, cy=c_y):
        img.draw_rectangle(tag.rect, color=(255, 0, 0))
        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
        x = tag.x_translation
        z = tag.z_translation

        found_tag = 1 # 1 if tag, 0 if no tag found

    msg = f'{found_tag},{x},{z}' # string to be sent over mqtt

    client.publish(topic_pub.encode(), msg.encode()) # publish

    found_tag = 0      # reset found_tag to 0
    time.sleep_ms(10
