# Theremin-main.py
# Runs on the Maker Pi Pico to control the Theremin, it responds to MQTT
# commands, as well as the two ultrasonic sensors and the light sensor.
# from these inputs, it sends notes over MIDI to the device it is connected to.

import asyncio
from machine import Pin, PWM, ADC
import utime
import time
import network
from BLE_CEEO import Yell
from mqtt import MQTTClient
from secrets import mysecrets
import math

PitchBend = 0xE0

MQTT_BROKER = 'broker.hivemq.com'
MQTT_PORT = 1883
TOPIC_SUB = mysecrets['topic']

max_distance = 100
client = None 

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(mysecrets['ssid'], mysecrets['password'])
    print("Connecting to Wi-Fi...", end='')
    while wlan.ifconfig()[0] == '0.0.0.0':
        print('.', end='')
        time.sleep(1)
    print("\nConnected to Wi-Fi")
    print(f"IP Address: {wlan.ifconfig()[0]}")

connect_wifi()

def callback(topic, msg):
    global max_distance
    msg_str = msg.decode()
    try:
        max_distance = int(msg_str)

        print(f"Received reverb value over MQTT: {msg_str}")
    except ValueError:
        print(f"Invalid MQTT message for reverb: {msg_str}")

def mqtt_connect():
    global client
    client = MQTTClient('Theremin', MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.set_callback(callback)
    client.connect()
    client.subscribe(TOPIC_SUB)
    print(f"Connected to MQTT broker at {MQTT_BROKER}, subscribed to topic '{TOPIC_SUB}'")

mqtt_connect()

async def check_mqtt():
    global client
    while True:
        try:
            client.check_msg()
        except OSError as e:
            print("MQTT Error")
            mqtt_connect()
        await asyncio.sleep(0.1)

# MIDI Message Constants
NoteOn = 0x90
NoteOff = 0x80

ControlChange = 0xB0  # Control Change command

# Initialize the BLE connection
p = Yell('Richard', verbose=True, type='midi')
p.connect_up()

# Define trigger and echo pins for the pitch sensor
trigger_pitch = Pin(11, Pin.OUT)
echo_pitch = Pin(10, Pin.IN)

# Define trigger and echo pins for the volume sensor
trigger_volume = Pin(14, Pin.OUT)
echo_volume = Pin(15, Pin.IN)

# ------------------------------
# 1. Motor Control Setup
# ------------------------------
# Define PWM pins for Motor 1 (Pitch Control)
MOTOR1_PWM_PIN_FORWARD = 2   # PWM pin for forward movement
MOTOR1_PWM_PIN_BACKWARD = 3  # PWM pin for backward movement

# Define PWM pins for Motor 2 (Volume Control)
MOTOR2_PWM_PIN_FORWARD = 4   # PWM pin for forward movement
MOTOR2_PWM_PIN_BACKWARD = 5  # PWM pin for backward movement

# Initialize PWM for Motor 1
motor1_forward_pwm = PWM(Pin(MOTOR1_PWM_PIN_FORWARD))
motor1_backward_pwm = PWM(Pin(MOTOR1_PWM_PIN_BACKWARD))

# Initialize PWM for Motor 2
motor2_forward_pwm = PWM(Pin(MOTOR2_PWM_PIN_FORWARD))
motor2_backward_pwm = PWM(Pin(MOTOR2_PWM_PIN_BACKWARD))

# Set PWM frequency (common for motor control)
PWM_FREQ = 1000  # 1 kHz

# Configure PWM for Motor 1
motor1_forward_pwm.freq(PWM_FREQ)
motor1_backward_pwm.freq(PWM_FREQ)
motor1_forward_pwm.duty_u16(0)
motor1_backward_pwm.duty_u16(0)

# Configure PWM for Motor 2
motor2_forward_pwm.freq(PWM_FREQ)
motor2_backward_pwm.freq(PWM_FREQ)
motor2_forward_pwm.duty_u16(0)
motor2_backward_pwm.duty_u16(0)

# ------------------------------
# 2. Configuration Parameters
# ------------------------------
# Motor 1 (Pitch Control) Parameters
MOTOR1_MIN_DISTANCE = 5     # Minimum distance in cm to start motor
MOTOR1_MAX_DISTANCE = 40    # Maximum distance in cm for full speed
MOTOR1_MIN_PWM = 10000      # Minimum PWM duty cycle to start motor (adjust as needed)
MOTOR1_MAX_PWM = 65535      # Maximum PWM duty cycle (0-65535)

# Motor 2 (Volume Control) Parameters
MOTOR2_MIN_DISTANCE = 5     # Minimum distance in cm to start motor
MOTOR2_MAX_DISTANCE = 40    # Maximum distance in cm for full speed
MOTOR2_MIN_PWM = 10000      # Minimum PWM duty cycle to start motor (adjust as needed)
MOTOR2_MAX_PWM = 65535      # Maximum PWM duty cycle (0-65535)

# Smoothing Parameters
SMOOTHING_FACTOR = 0.7  # EMA smoothing factor (0 < alpha < 1)
previous_distance_pitch = None
previous_distance_volume = None

# Initialize ADC on GPIO26 for light sensor
adc_light = ADC(Pin(26))

# Global state variables
last_note = None         # Tracks the last active MIDI note
last_volume = 0          # Tracks the last valid MIDI volume
light_over_threshold = False  # Indicates if light level is above 1000

async def monitor_light_task():
    global light_over_threshold, last_note

    while True:
        # Read the analog value (0-65535)
        analog_value = adc_light.read_u16()
        # print("Analog Light Level:", analog_value)

        if analog_value > 1000 and not light_over_threshold:
            print("Light level above 1000. Sending Note Off.")
            light_over_threshold = True
            
            # Send Note Off only once when threshold is crossed
            if last_note is not None:
                send_midi_message(NoteOff, last_note, 0)
                last_note = None

        elif analog_value < 1000 and light_over_threshold:
            light_over_threshold = False
            print("Light level below 1000. Resuming note functionality.")

        await asyncio.sleep(0.1)  # Check every 20ms

def ultra(trigger, echo, timeout_us=50000):
    """
    Measure distance using an ultrasonic sensor.

    Parameters:
        trigger (Pin): The trigger pin of the ultrasonic sensor.
        echo (Pin): The echo pin of the ultrasonic sensor.
        timeout_us (int): Timeout in microseconds to wait for the echo.

    Returns:
        float or None: The measured distance in centimeters, or None if timeout occurs.
    """
    trigger.low()
    utime.sleep_us(2)  # Ensure trigger is low
    trigger.high()
    utime.sleep_us(10)  # Send a 10us pulse
    trigger.low()

    signaloff, signalon = 0, 0  # Initialize variables

    # Wait for the echo pin to go high (start of echo)
    start = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout_us:
            return None  # Timeout reached
        signaloff = utime.ticks_us()

    start = utime.ticks_us()
    # Wait for the echo pin to go low (end of echo)
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout_us:
            return None  # Timeout reached
        signalon = utime.ticks_us()

    # Calculate the time passed and convert to distance
    if signaloff and signalon:
        timepassed = signalon - signaloff
        distance = (timepassed * 0.0343) / 2  # Speed of sound: 0.0343 cm/us
        return distance
    else:
        return None  # No valid reading

async def get_regular_distance(trigger, echo, samples=10, alpha=0.7):
    """
    Obtain a smoothed distance measurement using Exponential Moving Average (EMA).

    Parameters:
        trigger (Pin): The trigger pin of the ultrasonic sensor.
        echo (Pin): The echo pin of the ultrasonic sensor.
        samples (int): Number of samples to average.
        alpha (float): Smoothing factor between 0 and 1.

    Returns:
        float or None: The smoothed distance in centimeters, or None if no valid readings.
    """
    global previous_distance_pitch, previous_distance_volume
    smoothed_distance = None
    valid_readings = 0

    for _ in range(samples):
        distance = ultra(trigger, echo)
        if distance is not None:
            valid_readings += 1
            if smoothed_distance is None:
                smoothed_distance = distance
            else:
                smoothed_distance = alpha * distance + (1 - alpha) * smoothed_distance
        await asyncio.sleep(0.005)  # Short delay between readings

    if valid_readings == 0:
        return None
    return smoothed_distance

def send_midi_message(command, data1, data2):
    """
    Send a generic MIDI message.

    Parameters:
        command (int): MIDI command byte (e.g., NoteOn, NoteOff).
        data1 (int): First data byte (e.g., note number).
        data2 (int): Second data byte (e.g., velocity).
    """
    try:
        timestamp_ms = time.ticks_ms()
        tsM = (timestamp_ms >> 7 & 0b111111) | 0x80
        tsL = 0x80 | (timestamp_ms & 0x7F)

        payload = bytes([tsM, tsL, command, data1 & 0x7F, data2 & 0x7F])
        p.send(payload)
    except Exception as e:
        print("Failed to send MIDI message:", e)

def send_pitch_bend(bend_value):
    """
    Send a pitch bend MIDI message.

    Parameters:
        bend_value (int): Pitch bend value (0-16383).
    """
    try:
        bend_value = max(0, min(16383, bend_value))  # Clamp to valid range
        lsb = bend_value & 0x7F  # Least significant 7 bits
        msb = (bend_value >> 7) & 0x7F  # Most significant 7 bits
        send_midi_message(PitchBend, lsb, msb)
    except Exception as e:
        print("Failed to send pitch bend:", e)

def send_midi_cc(controller_number, value):
    """
    Send a MIDI Control Change message.

    Parameters:
        controller_number (int): MIDI controller number (e.g., 11 for expression).
        value (int): Controller value (0-127).
    """
    # Ensure value is within MIDI CC range
    value = max(0, min(127, value))
    send_midi_message(ControlChange, controller_number, value)

def theremin_control(base_note, pitch_bend, volume_value):
    """
    Adjust MIDI pitch bend and volume based on sensor inputs.

    Parameters:
        base_note (int): MIDI note number.
        pitch_bend (int): Pitch bend value (0-16383).
        volume_value (int): MIDI volume value (0-127).
    """
    send_pitch_bend(pitch_bend)
    send_midi_cc(11, volume_value)  # Using CC#11 for expression

def map_distance_to_pitch_and_bend(distance):
    """
    Map distance to a MIDI note and pitch bend value for smooth pitch transitions.

    Parameters:
        distance (float): Measured distance in centimeters.

    Returns:
        tuple: (base_note (int), pitch_bend (int))
    """
    min_distance = 0    # Minimum distance in cm

    global max_distance
    # max_distance = 100  # Maximum distance in cm

    # Clamp distance within bounds
    distance = max(min_distance, min(max_distance, distance))

    # Define MIDI note range
    min_note = 36  # C2
    max_note = 96  # C7

    # Total number of semitones in range
    total_semitones = max_note - min_note

    # Calculate exact pitch in semitones
    fractional_semitones = ((distance - min_distance) / (max_distance - min_distance)) * total_semitones
    exact_note = min_note + fractional_semitones

    # Split into base note and fractional part for pitch bend
    base_note = int(exact_note)
    fractional_part = exact_note - base_note

    # Map fractional part to pitch bend value
    bend_value = int(fractional_part * 16383)  # 0 to 16383

    return base_note, bend_value

def map_distance_to_volume(distance):
    """
    Map distance between 5 and 40 cm to MIDI volume (0-127).

    Parameters:
        distance (float): Measured distance in centimeters.

    Returns:
        int: MIDI volume value (0-127).
    """
    min_distance = 5     # Minimum reliable distance in cm
    max_distance = 60    # Maximum distance in cm

    # Clamp distance within bounds
    distance = max(min_distance, min(max_distance, distance))

    # Map distance to volume (farther is louder) scaled to 0-127
    volume_value = int(((distance - min_distance) / (max_distance - min_distance)) * 127)
    return volume_value

def map_distance_to_motor_speed(distance, motor_min_distance, motor_max_distance, motor_min_pwm, motor_max_pwm):
    """
    Map distance to motor PWM duty cycle, considering a threshold and minimum PWM.

    Parameters:
        distance (float): Measured distance in centimeters.
        motor_min_distance (int): Minimum distance in cm to start motor.
        motor_max_distance (int): Maximum distance in cm for full speed.
        motor_min_pwm (int): Minimum PWM duty cycle to start motor.
        motor_max_pwm (int): Maximum PWM duty cycle.

    Returns:
        int: PWM duty cycle value (0 - motor_max_pwm).
    """
    if distance <= motor_min_distance:
        return 0  # Motor stopped
    elif distance >= motor_max_distance:
        return motor_max_pwm  # Maximum speed
    else:
        # Linear mapping from motor_min_distance to motor_max_distance -> motor_min_pwm to motor_max_pwm
        speed = ((distance - motor_min_distance) / (motor_max_distance - motor_min_distance)) * (motor_max_pwm - motor_min_pwm) + motor_min_pwm
        return int(speed)

def set_motor_speed(pwm_value, motor_forward_pwm, motor_backward_pwm):
    """
    Set motor speed based on the PWM duty cycle value.

    Parameters:
        pwm_value (int): PWM duty cycle value (0 - motor_max_pwm).
        motor_forward_pwm (PWM): PWM object for forward movement.
        motor_backward_pwm (PWM): PWM object for backward movement.
    """
    if pwm_value > 0:
        # Move forward
        motor_forward_pwm.duty_u16(pwm_value)
        motor_backward_pwm.duty_u16(0)
    else:
        # Stop motor
        motor_forward_pwm.duty_u16(0)
        motor_backward_pwm.duty_u16(0)

async def measure_distance_task():
    """
    Asynchronous task to continuously measure distances and send MIDI signals.
    Also controls motor speeds based on pitch and volume distances.
    Music playback is disabled when light sensor reading is above threshold.
    """
    last_note = None
    last_volume = 0
    pitch_over_counter = 0
    pitch_over_threshold = 3  # Number of consecutive over_max readings to trigger NoteOff

    while True:
        # Check the light sensor threshold; if over, prevent note playback
        if light_over_threshold:
            # If the light level is above threshold, turn off any active note
            set_motor_speed(0, motor1_forward_pwm, motor1_backward_pwm)
            set_motor_speed(0, motor2_forward_pwm, motor2_backward_pwm)
            if last_note is not None:
                send_midi_message(NoteOff, last_note, 0)
                last_note = None
            print("Light sensor covered - notes off.")
        else:
            # Get distances from both sensors
            average_distance_pitch = await get_regular_distance(trigger_pitch, echo_pitch)
            average_distance_volume = await get_regular_distance(trigger_volume, echo_volume)

            # Handle pitch control and Motor 1
            if average_distance_pitch is not None:
                # Check if the distance indicates no hand is present (beyond max distance)
                if average_distance_pitch >= 100:
                    pitch_over_counter += 1
                    if pitch_over_counter >= pitch_over_threshold:
                        if last_note is not None:
                            send_midi_message(NoteOff, last_note, 0)
                            last_note = None
                        print("No hand detected for pitch control.")
                        # Stop Motor 1 since no hand is detected
                        set_motor_speed(0, motor1_forward_pwm, motor1_backward_pwm)
                else:
                    pitch_over_counter = 0  # Reset counter if a valid pitch reading is obtained
                    base_note, pitch_bend = map_distance_to_pitch_and_bend(average_distance_pitch)

                    # If base note has changed, send NoteOff for the last note and NoteOn for the new note
                    if last_note != base_note:
                        if last_note is not None:
                            send_midi_message(NoteOff, last_note, 0)
                        send_midi_message(NoteOn, base_note, 100)  # Send NoteOn with velocity 100
                        last_note = base_note

                    # Map volume distance to MIDI volume
                    if average_distance_volume is not None and 5 < average_distance_volume <= 40:
                        volume_value = map_distance_to_volume(average_distance_volume)
                        last_volume = volume_value  # Update last valid volume
                    else:
                        volume_value = last_volume  # Retain last valid volume if outlier

                    # Send pitch bend and volume messages
                    theremin_control(base_note, pitch_bend, volume_value)

                    # Control Motor 1 speed based on pitch distance
                    motor1_speed = map_distance_to_motor_speed(
                        average_distance_pitch,
                        MOTOR1_MIN_DISTANCE,
                        MOTOR1_MAX_DISTANCE,
                        MOTOR1_MIN_PWM,
                        MOTOR1_MAX_PWM
                    )
                    set_motor_speed(motor1_speed, motor1_forward_pwm, motor1_backward_pwm)

                    # Debug prints for Motor 1
                    # print(f"Pitch Distance: {average_distance_pitch:.2f} cm, Note: {base_note}, Pitch Bend: {pitch_bend}")
                    # print(f"Motor 1 Speed PWM: {motor1_speed}")

            else:
                pitch_over_counter += 1
                if pitch_over_counter >= pitch_over_threshold:
                    if last_note is not None:
                        send_midi_message(NoteOff, last_note, 0)
                        last_note = None
                    print("No valid pitch distance reading. No hand detected.")
                    # Stop Motor 1 since no valid reading
                    set_motor_speed(0, motor1_forward_pwm, motor1_backward_pwm)

            # Handle Volume Control and Motor 2
            if average_distance_volume is not None:
                if average_distance_volume >= 100:
                    # print("Volume distance exceeds operational range.")
                    set_motor_speed(0, motor2_forward_pwm, motor2_backward_pwm)
                else:
                    motor2_speed = map_distance_to_motor_speed(
                        average_distance_volume,
                        MOTOR2_MIN_DISTANCE,
                        MOTOR2_MAX_DISTANCE,
                        MOTOR2_MIN_PWM,
                        MOTOR2_MAX_PWM
                    )
                    set_motor_speed(motor2_speed, motor2_forward_pwm, motor2_backward_pwm)
                    # print(f"Volume Distance: {average_distance_volume:.2f} cm, Volume: {last_volume}")
                    # print(f"Motor 2 Speed PWM: {motor2_speed}")
            else:
                print("No valid volume distance reading. Retaining last valid volume.")

        await asyncio.sleep(0.02)  # Update every 20ms for smoother changes

async def main():
    """
    Main asynchronous function to run the distance measurement, monitor light, and MQTT check tasks concurrently.
    """
    await asyncio.gather(
        measure_distance_task(),
        monitor_light_task(),
        check_mqtt()
    )

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program interrupted by user.")
    # Ensure all notes are turned off upon exit
    if 'last_note' in locals() and last_note is not None:
        send_midi_message(NoteOff, last_note, 0)
    # Stop both motors
    set_motor_speed(0, motor1_forward_pwm, motor1_backward_pwm)
    set_motor_speed(0, motor2_forward_pwm, motor2_backward_pwm)

    p.disconnect()
