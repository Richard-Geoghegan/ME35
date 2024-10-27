# Dahal-Board-main.py
# Runs on the Dahal Board to control the height required 
# for your hand to reach the maximum pitch. This height is sent over MQTT
# to the Pico. The current height is shown on the onboard display and can be
# adjusted by twisting the potentiometer, and sent by pressing the button.

import asyncio
import time
from machine import Pin, SoftI2C, ADC
import network
from mqtt import MQTTClient  # Ensure you have the correct MQTTClient module
from secrets import mysecrets
import ssd1306

# Initialize I2C for OLED
i2c = SoftI2C(scl=Pin(7), sda=Pin(6))

# Initialize OLED Screen
screen = ssd1306.SSD1306_I2C(128, 64, i2c)

# Initialize Potentiometer
pot = ADC(Pin(3))
pot.atten(ADC.ATTN_11DB)  # Configure for full range (0-3.3V)

# Initialize Button
button = Pin(9, Pin.IN, Pin.PULL_UP)  # Assuming Pin 9 is the main button

class DahalBoard:
    MQTT_BROKER = 'broker.hivemq.com'
    MQTT_PORT = 1883
    TOPIC_PUB = mysecrets['topic']  # Topic to publish to

    def __init__(self):
        self.client = None
        self.is_connected = False
        self.last_distance_value = 100  # Default distance sensitivity shown at the top right
        self.current_distance_value = 0  # Displayed in the center as the current value

    def connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(mysecrets['ssid'], mysecrets['password'])
        print("Connecting to Wi-Fi...", end='')
        screen.fill(0)
        screen.text("Connecting to Wi-Fi...", 0, 20, 1)
        screen.show()
        while wlan.ifconfig()[0] == '0.0.0.0':
            print('.', end='')
            screen.text("." * (time.ticks_ms() // 500 % 5), 0, 30, 1)
            screen.show()
            time.sleep(1)
        print("\nConnected to Wi-Fi")
        print(f"IP Address: {wlan.ifconfig()[0]}")

    def mqtt_connect(self):
        try:
            self.client = MQTTClient('DahalBoard_Client', self.MQTT_BROKER, self.MQTT_PORT, keepalive=60)
            self.client.connect()
            print(f"Connected to MQTT broker at {self.MQTT_BROKER}")
            # Initial screen display
            screen.fill(0)
            screen.text("Distance Set:", 70, 0, 1)
            screen.text(f"{self.last_distance_value}", 70, 10, 1)  # Display default last set value at top right
            screen.show()
        except Exception as e:
            print("MQTT Connection Error:", e)
            screen.fill(0)
            screen.text("MQTT Connection Failed", 0, 20, 1)
            screen.show()
            time.sleep(2)
            self.mqtt_connect()  # Retry

    async def publish_distance_value(self, distance_value):
        """
        Publish the distance value over MQTT.
        """
        if self.client:
            try:
                self.client.publish(self.TOPIC_PUB, str(distance_value))
                print(f"Published distance value '{distance_value}' to topic '{self.TOPIC_PUB}'")
                # Update the last set value and display it at the top right
                self.last_distance_value = distance_value
                self.display_last_set_value()
                await self.show_sent_feedback()
            except Exception as e:
                print("Failed to publish MQTT message:", e)

    def display_current_value(self, value):
        """
        Display the current potentiometer reading in the center of the screen.
        """
        screen.fill_rect(20, 20, 128, 16, 0)  # Clear central area
        screen.text("Distance", 20, 20, 1)
        screen.text(f"{value}", 50, 30, 1)
        screen.show()

    def display_last_set_value(self):
        """
        Display the last set distance value at the top right of the screen.
        """
        screen.fill_rect(70, 0, 58, 16, 0)  # Clear top-right area
        screen.text(f"{self.last_distance_value}", 70, 0, 1)
        screen.show()

    async def show_sent_feedback(self):
        """
        Display "Sent" at the bottom of the screen for a short duration.
        """
        screen.fill_rect(0, 50, 128, 14, 0)  # Clear bottom area
        screen.text("Sent", 50, 50, 1)
        screen.show()
        await asyncio.sleep(1)  # Display "Sent" for 1 second
        screen.fill_rect(0, 50, 128, 14, 0)  # Clear "Sent" message
        screen.show()

    async def handle_user_input(self):
        while True:
            # Read potentiometer value
            pot_value = pot.read()  # Raw ADC value (0 - 4095)

            # Map potentiometer value to distance range (0-100)
            distance_value = int((pot_value / 4095) * 100)

            # Update the display with the current value in the center
            self.display_current_value(distance_value)

            # Check button press
            if not button.value():  # Assuming active low
                print(f"Button pressed. Setting distance: {distance_value}")
                await self.publish_distance_value(distance_value)
                # Debounce delay
                await asyncio.sleep(0.5)

            await asyncio.sleep(0.1)

    async def main(self):
        self.connect_wifi()
        self.mqtt_connect()
        await self.handle_user_input()

# Instantiate the DahalBoard
dahal_board = DahalBoard()

# Function to initialize and run the asyncio event loop
def run_asyncio_loop():
    try:
        asyncio.run(dahal_board.main())
    except KeyboardInterrupt:
        print("Program interrupted")

# Run the program
run_asyncio_loop()
