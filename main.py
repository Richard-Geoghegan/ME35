import asyncio
from machine import Pin, PWM
import network
import neopixel
from mqtt import MQTTClient
from secrets import mysecrets
import time

"""
The Motor class controls a motor via PWM (Pulse Width Modulation), manages a NeoPixel (RGB LED),
controls a buzzer, and handles MQTT communication for remote control of the motor's acceleration.
It also connects to Wi-Fi, subscribes to MQTT topics, and processes incoming MQTT messages.

Class Contract:
Inputs:
- pin_pwm (int): The GPIO pin number for motor control using PWM.

Outputs:
- The class initializes components (motor, NeoPixel, buzzer) and connects to Wi-Fi and MQTT broker.
- The class processes MQTT messages to control the motor's behavior (start, stop, adjust speed).
- The motor's acceleration is controlled via PWM signals.

Side Effects:
- Manages NeoPixel colors based on different states (Wi-Fi connected, MQTT connected, etc.).
- Connects to Wi-Fi and MQTT for remote control, and adjusts motor behavior accordingly.
- Continuously checks for incoming MQTT messages to adjust motor speed.
- Resets device states such as NeoPixel color and buzzer when deactivated.
"""
class Motor:

    NEOPIXEL_PIN = 28
    BUZZER_PIN = 18
    MQTT_BROKER = 'broker.hivemq.com'
    MQTT_PORT = 1883
    TOPIC_SUB = mysecrets['topic']
    BUTTON_PIN = 14
    LED_PIN = 12

    """
    Initializes the motor, NeoPixel, buzzer, and PWM controls.
    
    Inputs:
    - pin_pwm (int): GPIO pin number for motor control using PWM.
    
    Outputs:
    - Initializes motor PWM, NeoPixel, buzzer, and state variables.
    
    Side Effects:
    - Sets initial states for NeoPixel and motor, and prepares MQTT client.
    """
    def __init__(self, pin_pwm):
    
        # Initialize NeoPixel
        self.motorSide = "right"
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

        # State variables
        self.is_active = True     # Device starts as active
        self.client = None        # MQTT client

        self.pwm = PWM(Pin(pin_pwm))  # Initialize PWM on the given GPIO pin
        self.pwm.freq(1000)  # Set frequency to 1 kHz for motor control
        self.pwm.duty_u16(0)  # Start with the motor OFF (duty cycle = 0)

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


    """
    Connects to the MQTT broker and subscribes to a specific topic.
    
    Function Contract:
    - Inputs: None
    - Outputs: Establishes connection to the MQTT broker and subscribes to a topic.
    - Side effects: Updates NeoPixel color and prints connection status.
    """
    def mqtt_connect(self):
        self.client = MQTTClient('Motor_900000', self.MQTT_BROKER, self.MQTT_PORT, keepalive=60)
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
            asyncio.create_task(self.set_acceleration(0))
            self.is_active = False
            self.reset()
            print('Device deactivated via MQTT!')
        else: 
            left_speed_str, right_speed_str = msg_str.split(",")  # Split the message string into left and right speeds
    
            # Convert the string values to floats
            left_speed = float(left_speed_str)
            right_speed = float(right_speed_str)

            if self.is_active:
                # Set acceleration based on motorSide
                if self.motorSide == "left":
                    asyncio.create_task(self.set_acceleration(int(left_speed)))  # Set left motor acceleration
                else:
                    asyncio.create_task(self.set_acceleration(int(right_speed)))  # Set right motor acceleration
        
        print(f"Received MQTT message: {(topic.decode(), msg_str)}")


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
    Set the motor acceleration.
    Acceleration is a value between 0 (off) and 100 (full power).
    """
    async def set_acceleration(self, acceleration):
        # Ensure the value is within bounds (0 to 100)
        if acceleration < 0:
            acceleration = 0
        elif acceleration > 100:
            acceleration = 100

        # Convert the acceleration percentage to a PWM duty cycle
        duty_cycle = int(acceleration * 65535 / 100)
        self.pwm.duty_u16(duty_cycle)  # Set the PWM duty cycle for motor speed

        # For demonstration purposes, print the set acceleration
        print(f"Motor acceleration set to {acceleration}%")

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
    - Side effects: Starts Wi-Fi connection, MQTT connection, and event loops.
    """
    async def main(self):
        self.connect_wifi()
        self.mqtt_connect()
        await asyncio.gather(
            self.check_mqtt()
        )



# Instantiate and run the motor object
motor = Motor(26)
asyncio.run(motor.main())