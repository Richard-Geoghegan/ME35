import asyncio
from machine import I2C, Pin, PWM
import network
import neopixel
import struct
from mqtt import MQTTClient
from secrets import mysecrets
import time
import ntptime

"""
TapLight class that connects to Wi-Fi, initializes MQTT, monitors the accelerometer for taps,
changes the NeoPixel color, and buzzes upon tap detection.
"""
class TapLight:

    # Pin definitions
    SCL_PIN = 27
    SDA_PIN = 26
    NEOPIXEL_PIN = 28
    BUZZER_PIN = 18
    MQTT_BROKER = 'broker.hivemq.com'
    MQTT_PORT = 1883
    TOPIC_SUB = mysecrets['topic']
    BUTTON_PIN = 14
    LED_PIN = 12

    def __init__(self):
        # Initialize accelerometer
        self.accelerometer = self.Acceleration(self.SCL_PIN, self.SDA_PIN)

        # Initialize NeoPixel
        self.np = neopixel.NeoPixel(Pin(self.NEOPIXEL_PIN, Pin.OUT), 1)
        self.colors = [
            (0, 0, 0),
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (255, 0, 255),  # Magenta
            (255, 255, 255) # White
        ]
        self.current_color_index = 0

        # Set NeoPixel to maximum brightness at startup
        self.np[0] = self.colors[self.current_color_index]
        self.np.write()

        # Initialize Buzzer
        self.buzzer = PWM(Pin(self.BUZZER_PIN))
        self.buzzer.freq(1000)
        self.buzzer.duty_u16(0)  # Start with buzzer off

        # Tap detection parameters
        self.THRESHOLD = 1000     # Sensitivity threshold for tap detection
        self.debounce_time = 0.8  # 1 second debounce period
        self.tapped = False       # Flag to prevent multiple detections
        self.prev_accel = None    # Previous acceleration reading

        # State variables
        self.is_active = True     # Device starts as active
        self.client = None        # MQTT client

        self.alarm_time = None
        self.alarm_active = False

        self.button = Pin(self.BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        self.prev_button_state = self.button.value()

        # Initialize the external LED
        self.led = Pin(12, Pin.OUT)
        self.led.on()  # Turn the external LED on at startup

        # self.topic_pub = 'ME35-24/minecraft'
        self.pub_msg = 'buzz'

    """
    Connects to the Wi-Fi network using SSID and password from the secrets file.
    
    Function Contract:
    - Inputs: None
    - Outputs: Connects the device to Wi-Fi and sets the system time.
    - Side effects: Turns on the NeoPixel color after successful connection.
    """
    def connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(mysecrets['ssid'], mysecrets['password'])
        print("Connecting to Wi-Fi...", end='')
        while wlan.ifconfig()[0] == '0.0.0.0':
            print('.', end='')
            time.sleep(1)
        self.np[0] = self.colors[7]
        self.np.write()
        time.sleep(0.5)
        self.np[0] = self.colors[0]
        self.np.write()
        print("\nConnected to Wi-Fi")
        print(f"IP Address: {wlan.ifconfig()[0]}")
        ntptime.settime()

    """
    Asynchronously checks the button state and toggles activation when pressed.
    
    Function Contract:
    - Inputs: None
    - Outputs: None (runs continuously)
    - Side effects: Toggles the device's `is_active` state and manages the external LED.
    """
    async def check_button(self):
        while True:
            current_button_state = self.button.value()
            if self.prev_button_state and not current_button_state:  # Button press detected
                print('Button pressed!')
                # Toggle the `is_active` state
                self.is_active = not self.is_active

                if self.is_active:
                    print('Activating...')
                    self.led.on()  # Turn on the external LED when activated
                else:
                    print('Deactivating...')
                    self.reset()  # Reset and turn off the LED when deactivated

            self.prev_button_state = current_button_state
            await asyncio.sleep(0.1)


    """
    Connects to the MQTT broker and subscribes to a specific topic.
    
    Function Contract:
    - Inputs: None
    - Outputs: Establishes connection to the MQTT broker and subscribes to a topic.
    - Side effects: Updates NeoPixel color and prints connection status.
    """
    def mqtt_connect(self):
        self.client = MQTTClient('TapLight_Client', self.MQTT_BROKER, self.MQTT_PORT, keepalive=60)
        self.client.set_callback(self.callback)
        self.client.connect()
        self.client.subscribe(self.TOPIC_SUB)
        print(f"Connected to MQTT broker at {self.MQTT_BROKER}, subscribed to topic '{self.TOPIC_SUB}'")
        self.np[0] = self.colors[2]
        self.np.write()
        time.sleep(0.5)
        self.np[0] = self.colors[0]
        self.np.write()

    """
    Handles incoming MQTT messages and manages the device state based on the message content.
    
    Function Contract:
    - Inputs: `topic` (MQTT topic), `msg` (message payload)
    - Outputs: Processes the message, updates device state, and triggers actions.
    - Side effects: Controls alarm, activates/deactivates device, and sets alarm time.
    """
    def callback(self, topic, msg):
        msg_str = msg.decode()
        if msg_str == 'start':
            self.is_active = True
            print('Device activated via MQTT!')
        elif msg_str == 'stop':
            self.is_active = False
            self.reset()
            print('Device deactivated via MQTT!')
        elif ':' in msg_str:
            asyncio.create_task(self.beep(0.5))
            print(f"Received time: {msg_str}")
            self.alarm_time = msg_str
        elif msg_str == 'something':
            asyncio.create_task(self.beep(1.5))
            
        print(f"Received MQTT message: {(topic.decode(), msg_str)}")

    """
    Asynchronously monitors time and triggers an alarm when the alarm time is reached.
    
    Function Contract:
    - Inputs: None
    - Outputs: Continuously checks if the current time matches the alarm time.
    - Side effects: Activates alarm and changes NeoPixel color when the alarm is triggered.
    """
    async def countdown(self):
        while True:
            if self.alarm_time is not None:
                timezone_offset = -4 * 3600
                hour, minute = map(int, self.alarm_time.split(':'))
                current_time = time.localtime(time.time() + timezone_offset)
                current_hour, current_minute = current_time[3], current_time[4]
                if (current_hour == hour) and (current_minute == minute):
                    self.alarm_active = True
                    print("Alarm!!")
                    while self.alarm_active:
                        self.np[0] = self.colors[1]
                        self.np.write()
                        asyncio.create_task(self.beep(0.2))
                        await asyncio.sleep(0.3)
                        self.np[0] = self.colors[0]
                        self.np.write()
                        await asyncio.sleep(0.2)
            await asyncio.sleep(1)
            

    """
    Asynchronously checks for incoming MQTT messages in a loop.
    
    Function Contract:
    - Inputs: None
    - Outputs: Continuously monitors MQTT messages.
    - Side effects: Reconnects to MQTT if a connection error occurs.
    """
    async def check_mqtt(self):
        while True:
            try:
                self.client.check_msg()
            except OSError as e:
                print("MQTT Error")
                self.mqtt_connect()
            await asyncio.sleep(0.1)

    """
    Asynchronously monitors accelerometer for tap events and changes NeoPixel color upon detection.
    
    Function Contract:
    - Inputs: None
    - Outputs: Monitors and responds to tap events by changing NeoPixel colors.
    - Side effects: Publishes a message to MQTT when a tap is detected.
    """
    async def monitor_accelerometer(self):
        while True:
            try:
                accel = self.accelerometer.read_accel()
                if self.prev_accel is not None:
                    # Calculate the difference between current and previous readings
                    dx = accel[0] - self.prev_accel[0]
                    dy = accel[1] - self.prev_accel[1]
                    dz = accel[2] - self.prev_accel[2]
                    delta = (dx**2 + dy**2 + dz**2)**0.5  # Euclidean distance
                    if delta > self.THRESHOLD and not self.tapped and self.is_active:
                        self.client.publish(self.TOPIC_SUB.encode(),self.pub_msg.encode())
                        if self.alarm_active:
                            self.alarm_active = False
                            self.alarm_time = None
                            self.np[0] = self.colors[0]
                            self.np.write()
                        print("TAPPED")
                        # Change to the next color
                        self.current_color_index = (self.current_color_index + 1) % len(self.colors)
                        self.np[0] = self.colors[self.current_color_index]
                        self.np.write()
                        # Buzz after tap
                        asyncio.create_task(self.beep(0.2))
                        self.tapped = True
                        # Start debounce timer
                        asyncio.create_task(self.debounce())
                self.prev_accel = accel
            except Exception as e:
                print(f"Error reading acceleration: {e}")
            await asyncio.sleep(0.05)


    """
    Prevents multiple tap detections within a set debounce time.

    Function Contract:
    - Inputs: None
    - Outputs: Waits for the debounce time to expire.
    - Side effects: Resets the `tapped` flag to allow new taps to be detected.
    """
    async def debounce(self):
        await asyncio.sleep(self.debounce_time)
        self.tapped = False

    """
    Produces a beep sound using the buzzer for a specified duration.

    Function Contract:
    - Inputs: `duration` (float) – The duration of the beep in seconds.
    - Outputs: Buzzer turns on and off after the specified time.
    - Side effects: Changes the buzzer's duty cycle to generate the sound.
    """
    async def beep(self, duration):
        self.buzzer.duty_u16(32768)  # Set duty cycle to 50%
        await asyncio.sleep(duration)
        self.buzzer.duty_u16(0)      # Turn off buzzer

    """
    Resets the device state, turning off the NeoPixel and buzzer, and resetting the LED.

    Function Contract:
    - Inputs: None
    - Outputs: Resets the device's color, buzzer, and LED state.
    - Side effects: Turns off all indicators and ensures the buzzer is silent.
    """
    def reset(self):
        self.current_color_index = 0
        self.np[0] = self.colors[self.current_color_index]
        self.np.write()
        self.buzzer.duty_u16(0)  # Ensure buzzer is off
        self.led.off()

    """
    The main loop that orchestrates all device tasks asynchronously.

    Function Contract:
    - Inputs: None
    - Outputs: Runs the core functions concurrently to manage Wi-Fi, MQTT, and sensors.
    - Side effects: Starts Wi-Fi connection, MQTT connection, and event loops for the button, accelerometer, and alarm.
    """
    async def main(self):
        self.connect_wifi()
        self.mqtt_connect()
        await asyncio.gather(
            self.check_mqtt(),
            self.monitor_accelerometer(),
            self.countdown(),
            self.check_button()
        )

    """
    A helper class to manage accelerometer operations, such as reading data via I2C.

    Function Contract:
    - Inputs: I2C pin numbers and accelerometer address.
    - Outputs: Reads and returns accelerometer data.
    - Side effects: Initializes and manages communication with the accelerometer over I2C.
    """
    class Acceleration:
        def __init__(self, scl_pin_number, sda_pin_number, addr=0x62):
            scl_pin = Pin(scl_pin_number)
            sda_pin = Pin(sda_pin_number)
            self.addr = addr
            self.i2c = I2C(1, scl=scl_pin, sda=sda_pin, freq=100000)
            self.connected = False
            if self.is_connected():
                print('Accelerometer connected')
                self.write_byte(0x11, 0)

        """
        Checks if the accelerometer is connected via I2C.

        Function Contract:
        - Inputs: None
        - Outputs: Returns a boolean indicating the connection status.
        - Side effects: Scans for I2C devices and sets the connection flag.
        """
        def is_connected(self):
            options = self.i2c.scan()
            print(f"Available I2C devices: {options}")
            self.connected = self.addr in options
            return self.connected

        """
        Reads accelerometer data over I2C.

        Function Contract:
        - Inputs: None
        - Outputs: Returns acceleration data in x, y, z as a tuple.
        - Side effects: Communicates with the I2C device to retrieve acceleration values.
        """
        def read_accel(self):
            buffer = self.i2c.readfrom_mem(self.addr, 0x02, 6)
            return struct.unpack('<hhh', buffer)

        """
        Sends a command byte to the accelerometer over I2C.

        Function Contract:
        - Inputs: `cmd` (int), `value` (int) – Command and value to write to the device.
        - Outputs: Writes a command to the I2C device.
        - Side effects: Modifies the internal state of the accelerometer.
        """
        def write_byte(self, cmd, value):
            self.i2c.writeto_mem(self.addr, cmd, value.to_bytes(1, 'little'))

# Instantiate and run the TapLight object
tap_light = TapLight()
asyncio.run(tap_light.main())