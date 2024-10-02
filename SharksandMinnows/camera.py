import time
import math
import network
from mqtt import MQTTClient
import sensor
import uasyncio as asyncio

class AprilTagSteeringMQTT:
    def __init__(self, ssid, key, mqtt_broker, mqtt_port, mqtt_topic, interval=0.5, min_size=0, max_size=30):
        self.ssid = ssid
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_brightness(0)  # Adjust if needed (-3 to 3, 0 is neutral)
        sensor.set_contrast(3)    # Increase contrast (range: -3 to 3, 3 is max)
        sensor.set_saturation(-3) #
        # Initialize the camera settings with optimized resolution
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)  # Switch to black-and-white mode
        sensor.set_framesize(sensor.QQVGA)  # Lower resolution improves FPS (try QQVGA or even lower)
        sensor.skip_frames(time=2000)
        
        # Ensure manual settings for gain and white balance
        sensor.set_auto_gain(False)  # Disable auto gain
        sensor.set_auto_whitebal(False)  # Disable auto white balance
    
        # Adjust brightness and contrast
        sensor.set_brightness(0)  # Adjust brightness (-3 to 3, 0 is neutral)
        sensor.set_contrast(3)    # Increase contrast (range: -3 to 3, 3 is max)
        sensor.set_saturation(-3) # Decrease saturation for better black-and-white contrast
    
        # Refresh the preview to apply the changes immediately
        sensor.skip_frames(time=500)  # Wait for frames to adjust to new settings
        self.key = key
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_topic = mqtt_topic
        self.interval = interval  # Time interval between MQTT messages (in seconds)
        self.min_size = min_size  # Minimum size to map speed (size = 0 -> speed = 0)
        self.max_size = max_size  # Maximum size to map speed (size = max_size -> speed = 100)
        self.wlan = network.WLAN(network.STA_IF)
        self.client = None  # MQTT client object
        self.connect_to_wifi()
        self.mqtt_connect()
        self.init_camera()
        self.last_published_time = time.ticks_ms()  # Track the last time MQTT data was published

    def connect_to_wifi(self):
        # Initialize WLAN and connect to the network
        self.wlan.active(True)
        self.wlan.connect(self.ssid, self.key)

        while not self.wlan.isconnected():
            print(f'Trying to connect to "{self.ssid}"...')
            time.sleep_ms(1000)
        
        print("WiFi Connected:", self.wlan.ifconfig())

    def mqtt_connect(self):
        # Initialize and connect to MQTT
        try:
            self.client = MQTTClient("openmv", self.mqtt_broker, port=self.mqtt_port)
            self.client.connect()
            print("MQTT connected.")
        except OSError as e:
            print("MQTT connection failed, retrying...")
            time.sleep_ms(1000)
            self.mqtt_connect()  # Retry connection if it fails

    def init_camera(self):
        # Initialize the camera settings with optimized resolution
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)  # Lower resolution improves FPS (try QQVGA or even lower)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

    async def check_mqtt(self):
        # Check the MQTT connection and handle errors
        while True:
            try:
                self.client.check_msg()
            except OSError as e:
                print("MQTT Error, reconnecting...")
                self.mqtt_connect()  # Reconnect on error
            await asyncio.sleep(0.1)

    def calculate_motor_speeds(self, tag_size, tag_rotation):
        """
        Calculate the left and right motor speeds based on tag size and rotation.
        Size controls overall speed, rotation controls steering.
        """
        print(f"Detected size: {tag_size}, rotation: {tag_rotation}")
    
        # Handle size to determine forward/backward and acceleration
        if tag_size < 18:
            # If the tag is too small, move backward at a speed relative to how small it is
            speed = -50  # Set backward speed when below minimum size
        elif tag_size >= 70:
            speed = 100  # Maximum forward speed
        elif tag_size < 38:
            # Linearly map size to backward speed, with base (38) as 0 and min (18) as -50
            speed = (tag_size - 38) / (18 - 38) * -100  # Moving backward
        else:
            # Linearly map size to forward speed, with base (38) as 0 and max (70) as 100
            speed = (tag_size - 38) / (70 - 38) * 100  # Moving forward
    
        # Normalize the rotation angle to the range [0, 360)
        tag_rotation = tag_rotation % 360
    
        # Simplified Steering Logic
        if 349 <= tag_rotation <= 365 or 0 <= tag_rotation <= 8:
            # Neutral zone, drive straight
            left_motor_speed = speed
            right_motor_speed = speed
        elif 8 < tag_rotation <= 45:  # Aggressive left turn between 8 and 45 degrees
            left_motor_speed = speed * 3
            right_motor_speed = speed * 0.2  # Significantly reduce right motor speed for left turns
            print(f"Left Turn Detected, Angle: {tag_rotation}, Left Motor: {left_motor_speed}, Right Motor: {right_motor_speed}")
        elif 315 <= tag_rotation < 349:  # Aggressive right turn between 315 and 349 degrees
            left_motor_speed = speed * 0.2  # Significantly reduce left motor speed for right turns
            right_motor_speed = speed
            print(f"Right Turn Detected, Angle: {tag_rotation}, Left Motor: {left_motor_speed}, Right Motor: {right_motor_speed}")
        else:
            # For rotations outside the bounds, drive straight
            left_motor_speed = speed
            right_motor_speed = speed
    
        # Clip speeds to [-100, 100] range (to account for backward movement)
        left_motor_speed = max(min(left_motor_speed, 100), -100)
        right_motor_speed = max(min(right_motor_speed, 100), -100)
    
        # Print the motor speeds for debugging
        print(f"Calculated Motor Speeds -> Left: {left_motor_speed}, Right: {right_motor_speed}, Angle: {tag_rotation} degrees")
    
        return left_motor_speed, right_motor_speed
      
    async def publish_motor_speeds(self, left_speed, right_speed):
        """
        Publish the motor speeds to MQTT.
        """
        message = f"{left_speed},{right_speed}"
        print("Publishing:", message)

        try:
            # Publish the motor speeds to the MQTT topic
            self.client.publish(self.mqtt_topic, message)
        except OSError as e:
            print("MQTT publish failed, reconnecting...")
            self.mqtt_connect()  # Reconnect if publish fails

    async def process_tags(self):
        # Capture and process AprilTags
        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()  # Capture an image

            for tag in img.find_apriltags():
                # Calculate the size and rotation of the tag
                corners = tag.corners()
                tag_size = math.sqrt((corners[0][0] - corners[2][0]) ** 2 + (corners[0][1] - corners[2][1]) ** 2)
                tag_rotation = (180 * tag.rotation()) / math.pi

                # Print the detected tag angle in degrees
#                print(f"Detected Tag Rotation: {tag_rotation} degrees")

                # Only publish if the interval has passed
                current_time = time.ticks_ms()
                if time.ticks_diff(current_time, self.last_published_time) >= self.interval * 1000:
                    self.last_published_time = current_time
                    left_speed, right_speed = self.calculate_motor_speeds(tag_size, tag_rotation)
                    await self.publish_motor_speeds(left_speed, right_speed)

            await asyncio.sleep(0)  # Yield to allow other tasks to run

    async def run(self):
        # Start both the MQTT checker and AprilTag processor tasks
        mqtt_task = asyncio.create_task(self.check_mqtt())
        tag_task = asyncio.create_task(self.process_tags())
        await asyncio.gather(mqtt_task, tag_task)

# Main function to run the code
async def main():
    ssid = "Tufts_Robot"
    key = ""
    mqtt_broker = "broker.hivemq.com"
    mqtt_port = 1883
    mqtt_topic = "ME35-24/Richard"

    publisher = AprilTagSteeringMQTT(ssid, key, mqtt_broker, mqtt_port, mqtt_topic, interval=0.1, min_size=70, max_size=130)
    await publisher.run()

# Run the asyncio event loop
asyncio.run(main())
