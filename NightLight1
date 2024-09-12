import asyncio, neopixel, network, time
from machine import Pin, PWM
from mqtt import MQTTClient
from secrets import mysecrets

"""
NightLight class that controls an IoT device with RGB lights, buzzer, LED breathing effect, 
and MQTT-based remote control.

Attributes:
    mqtt_broker (str): The MQTT broker address.
    port (int): The port to use for the MQTT broker.
    topic_sub (str): The topic to subscribe to for MQTT messages.
    buzz (PWM): The buzzer, controlled by PWM.
    state (tuple): The current RGB state for the NeoPixel light.
    light (NeoPixel): The NeoPixel object controlling the RGB light.
    led (PWM): The LED controlled by PWM for breathing effect.
    is_active (bool): State to track whether the device is active or not.
"""
class NightLight:
    BUZZER_PIN = 18
    LED_PIN = 0
    BUTTON_PIN = 20
    NEOPIXEL_PIN = 28
    MQTT_BROKER = 'broker.hivemq.com'
    MQTT_PORT = 1883
    TOPIC_SUB = mysecrets['topic']
    
    """
    Initializes the NightLight object by setting up the MQTT broker, PWM for the buzzer and LED, 
    and the NeoPixel light with a default RGB state.
    """
    def __init__(self):
        self.buzz = PWM(Pin(self.BUZZER_PIN, Pin.OUT))
        self.buzz.freq(440)
        self.state = (100, 0, 100)
        self.light = neopixel.NeoPixel(Pin(self.NEOPIXEL_PIN), 1)
        self.light[0] = self.state
        self.light.write()
        self.led = PWM(Pin(self.LED_PIN, Pin.OUT))
        self.led.freq(50)
        self.is_active = False
    
    """
    Connects the device to the Wi-Fi network using SSID and password retrieved from the secrets file.
    
    This method keeps trying to connect until a valid IP address is obtained via DHCP.
    
    Returns:
        tuple: The device's IP configuration as a tuple of strings.
    """
    def connect(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(mysecrets['ssid'], mysecrets['password'])
        while wlan.ifconfig()[0] == '0.0.0.0':
            print('.', end=' ')
            time.sleep(1)
        print(wlan.ifconfig())
    
    """
    Resets the buzzer, NeoPixel, and LED states by turning them off (no sound or light).
    """
    def reset(self):
        self.buzz.duty_u16(0)
        self.light[0] = (0, 0, 0)
        self.light.write()
        self.led.duty_u16(0)
    
    """
    MQTT callback function that processes incoming messages to control the device.
    
    When a 'start' message is received, the device is activated. When a 'stop' message is received,
    the device is deactivated and reset.
    
    Args:
        topic (bytes): The topic on which the message was received.
        msg (bytes): The message payload.
    """
    def callback(self, topic, msg):
        msg_str = msg.decode()
        if msg_str == 'start':
            self.is_active = True
            print('Activated!')
        elif msg_str == 'stop':
            self.is_active = False
            print('Off!')
        print(f"Received: {(topic.decode(), msg_str)}")

    """
    Connects to the MQTT broker and subscribes to the specified topic.
    
    Returns:
        MQTTClient: The connected MQTT client object.
    """
    def mqtt_connect(self):
        client = MQTTClient('ME35_chris', self.MQTT_BROKER, self.MQTT_PORT, keepalive=60)
        client.connect()
        print(f"Connected to {self.MQTT_BROKER}")
        client.set_callback(self.callback)
        client.subscribe(self.TOPIC_SUB)
        return client

    """
    Updates the NeoPixel light's RGB state by incrementing each color value.
    After each update, the NeoPixel is refreshed with the new color.
    """
    def update_state(self):
        # Calculate the new RGB value for the Neopixel
        state_list = [(x + 100) if (x + 100) <= 255 else 0 for x in self.state]
        self.state = tuple(state_list)
        self.light[0] = self.state
        self.light.write()
        self.beep(1000)

    """
    Activates the buzzer for a short duration and then turns it off.

    Args:
        beep_dc (int): Duty cycle value for the buzzer's PWM (0-65535).
    """
    def beep(self, beep_dc):
        self.buzz.duty_u16(beep_dc)
        time.sleep(0.5)
        self.buzz.duty_u16(0)
    
    """
    Asynchronously checks for MQTT messages in a loop.
    This method continuously polls the MQTT broker and handles incoming messages.
    """
    async def check_mqtt(self):
        client = self.mqtt_connect()
        while True:
            client.check_msg()
            await asyncio.sleep(0.1)

    """
    Asynchronously checks the button press state in a loop.
    When the button is pressed, the NeoPixel's state is updated and the buzzer is activated.
    The method also resets the device when inactive.
    """
    async def check_btn(self):
        btn = Pin(self.BUTTON_PIN, Pin.IN)
        prev_state = btn.value()
        while True:
            if self.is_active:
                current_state = btn.value()
                if not prev_state and current_state:
                    print('Button released!')
                    self.update_state()
                prev_state = current_state
            else:
                self.reset()
            await asyncio.sleep(0.01)

    """
    Asynchronously runs a breathing LED effect when the device is active.
    The brightness of the LED gradually increases and decreases in a loop to simulate a breathing effect.
    The device is reset when inactive.
    """
    async def breathe(self):
        while True:
            if self.is_active:
                # Increase brightness
                for brightness in range(0, 65535, 500):
                    self.led.duty_u16(brightness)
                    await asyncio.sleep(0.01)
                # Decrease brightness
                for brightness in range(65535, 0, -500):
                    self.led.duty_u16(brightness)
                    await asyncio.sleep(0.01)
            else:
                self.reset()
            await asyncio.sleep(0.01)

    """
    The main asynchronous loop that manages the device's operation.
    This method gathers all the tasks (MQTT message checking, button monitoring, and LED breathing effect)
    and runs them concurrently.
    """
    async def main(self):
        self.connect()
        await asyncio.gather(self.check_mqtt(), self.check_btn(), self.breathe())

# Instantiate and run the NightLight object
nl = NightLight()
asyncio.run(nl.main())
