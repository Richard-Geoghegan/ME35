import asyncio
import time
import os
from machine import Pin, PWM
from Tufts_ble import Sniff, Yell
import neopixel

"""
Class representing a player in a Zombie game, which can either be a human or a zombie. 
The class manages game mechanics such as proximity detection, tagging, and role transitions.

Attributes:
    role (str): The player's role ('human' or 'zombie').
    zombie_number (int): The zombie's assigned number.
    max_zombie_number (int): The maximum valid zombie number.
    rssi_threshold (int): The RSSI threshold for proximity detection.
    proximity_duration (int): Duration in seconds required to tag a human.
    verbose (bool): Controls whether to output verbose logs.
    is_game_over (bool): Flag to track if the game is over for this player.
    tag_counts (dict): Tracks the number of times tagged by each zombie.
    proximity_states (dict): Tracks proximity state for each zombie.
"""
class Zombie:

    BUZZER_PIN = 18
    WARNING_PIN = 10
    NEOPIXEL_PIN = 28
    EYE_PIN = 9

    """
    Initializes the Zombie game instance.

    :param role: 'human' or 'zombie'
    :param zombie_number: Number assigned to the zombie (1 to max_zombie_number)
    :param max_zombie_number: Maximum valid zombie number (default is 13)
    :param rssi_threshold: RSSI threshold for proximity
    :param proximity_duration: Duration in seconds to stay within range to register a tag
    :param verbose: Enable verbose output
    """
    def __init__(self, role='human', zombie_number=8, max_zombie_number=13, rssi_threshold=-60, proximity_duration=3, verbose=True):
        self.role = role  # 'human' or 'zombie'
        self.zombie_number = zombie_number  # Assigned number if zombie
        self.max_zombie_number = max_zombie_number  # Maximum valid zombie number
        self.rssi_threshold = rssi_threshold
        self.proximity_duration = proximity_duration  # Time required within range to register a tag
        self.verbose = verbose
        self.is_game_over = False
        self.tag_counts = {}  # Tracks the number of times tagged by each zombie
        self.proximity_states = {}  # Tracks proximity states for each zombie
        self.humanStartTime = time.time()
        self.buzzer = PWM(Pin(self.BUZZER_PIN))
        self.buzzer.freq(1000)
        self.buzzer.duty_u16(0)  # Start with buzzer off

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

        self.warningLed = Pin(self.WARNING_PIN, Pin.OUT)
        self.eyeLed = Pin(self.EYE_PIN, Pin.OUT)

        self.np = neopixel.NeoPixel(Pin(self.NEOPIXEL_PIN, Pin.OUT), 1)
        self.np[0] = self.colors[7]
        self.np.write()

        if self.role == 'zombie':
            if self.zombie_number is None:
                raise ValueError(f"Zombie must have a zombie_number between 1 and {self.max_zombie_number}.")
            if not (1 <= self.zombie_number <= self.max_zombie_number):
                raise ValueError(f"Zombie number must be between 1 and {self.max_zombie_number}.")
            self.advertiser = Yell()
        elif self.role == 'human':
            self.scanner = Sniff(discriminator='!', verbose=False)
        else:
            raise ValueError("Role must be 'human' or 'zombie'.")

    """
    Starts the game loop for the device based on the role (human or zombie).
    Continuously advertises or scans based on the role.

    :raises asyncio.CancelledError: If the game loop is cancelled.
    """
    async def run(self):
        if self.role == 'zombie':
            await self.run_zombie()
        elif self.role == 'human':
            await self.run_human()
    
    """
    Writes tag counts data to a file.

    :param data: List of dictionaries containing 'zombie_number' and 'tag_count'.
    """
    def write_to_file(self, data):
        with open('tag_data.txt', 'w') as f:
            for entry in data:
                f.write(f"Zombie {entry['zombie_number']} tagged {entry['tag_count']} times.\n")

    """
    Makes the buzzer sound for a given duration.

    :param duration: Time in seconds for the buzzer to sound.
    """
    async def beep(self, duration):
        self.buzzer.duty_u16(32768)  # Set duty cycle to 50%
        await asyncio.sleep(duration)
        self.buzzer.duty_u16(0)      # Turn off buzzer

    """
    Zombie mode: Continuously advertises the zombie's presence to nearby devices.

    :raises asyncio.CancelledError: If the advertising loop is cancelled.
    """
    async def run_zombie(self):
        self.np[0] = self.colors[1]
        self.np.write()
        self.eyeLed.on()
        self.warningLed.off()
        zombie_name = f"!{self.zombie_number}"
        if self.verbose:
            print(f"Zombie {self.zombie_number} started advertising as {zombie_name}")
        self.advertiser.advertise(name=zombie_name)
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            self.advertiser.stop_advertising()
            if self.verbose:
                print("Zombie stopped advertising.")

    """
    Human mode: Scans for nearby zombies and manages tagging logic.

    :raises asyncio.CancelledError: If the scanning loop is cancelled.
    """
    async def run_human(self):

        self.np[0] = self.colors[2]
        self.np.write()
        if self.verbose:
            print("Human started scanning for zombies.")
        self.scanner.scan(0)  # Start scanning indefinitely
        try:
            while not self.is_game_over:
                # Process any new advertisements
                zombie_name = self.scanner.last_name
                rssi = self.scanner.last_rssi
                if zombie_name and rssi is not None:
                    # Clear the last scanned data
                    self.scanner.last_name = ''
                    self.scanner.last_rssi = None

                    # Check if the advertiser is a valid zombie within the RSSI threshold
                    if self.is_valid_zombie(zombie_name, rssi):
                        zombie_number = int(zombie_name[1:])
                        await self.update_proximity(zombie_number)
                # Periodically check proximity states
                await self.check_proximity()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            self.scanner.stop_scan()
            if self.verbose:
                print("Human stopped scanning.")

    """
    Checks if the scanned device is a valid zombie within the RSSI threshold.

    :param name: Advertised name
    :param rssi: Received signal strength
    :return: True if valid zombie, False otherwise
    """
    def is_valid_zombie(self, name, rssi):
        if name.startswith('!') and name[1:].isdigit():
            zombie_number = int(name[1:])
            if 1 <= zombie_number <= self.max_zombie_number and rssi >= self.rssi_threshold:
                if self.verbose:
                    print(f"Detected zombie {name} with RSSI {rssi}")
                return True
        return False

    """
    Updates the proximity state of a specific zombie.

    :param zombie_number: The number of the zombie detected within range.
    """
    async def update_proximity(self, zombie_number):
        current_time = time.time()
        state = self.proximity_states.get(zombie_number, {
            'in_range': False,
            'last_seen_time': None,
            'proximity_start_time': None,
            'tagged': False  # Track if this zombie has already tagged us
        })

        state['last_seen_time'] = current_time

        if not state['in_range']:
            # Zombie has just entered the range
            state['in_range'] = True
            state['proximity_start_time'] = current_time
            state['tagged'] = False  # Reset tagged state when zombie re-enters
            if self.verbose:
                print(f"Entered range of zombie {zombie_number}")
        self.proximity_states[zombie_number] = state

    """
    Periodically checks the proximity state of all zombies and handles state changes, 
    including turning LEDs on or off and processing tags.

    :raises asyncio.CancelledError: If the proximity check loop is cancelled.
    """
    async def check_proximity(self):
        current_time = time.time()
        out_of_range_threshold = 1  # Time in seconds to consider out of range
        zombies_to_remove = []

        for zombie_number, state in self.proximity_states.items():
            last_seen = state['last_seen_time']
            if current_time - last_seen > out_of_range_threshold:
                # Zombie is out of range
                if state['in_range']:
                    state['in_range'] = False
                    state['proximity_start_time'] = None
                    # self.warningLed.off()
                    if self.verbose:
                        print(f"Exited range of zombie {zombie_number}")
                    state['tagged'] = False  # Require re-entry for next tag
                # Remove if not seen for a longer time
                if current_time - last_seen > out_of_range_threshold * 5:
                    zombies_to_remove.append(zombie_number)
            else:
                # Zombie is in range and has not yet tagged us
                if state['in_range'] and not state['tagged']:
                    proximity_time = current_time - state['proximity_start_time']
                    # self.warningLed.on()
                    if proximity_time >= self.proximity_duration:
                        # Time threshold met, register a tag
                        await self.handle_tagging(zombie_number)
                        # Mark this zombie as having tagged us
                        state['tagged'] = True  # Prevent further tags until re-entry
                        if self.verbose:
                            print(f"Zombie {zombie_number} tagged after {self.proximity_duration} seconds in range.")

        if any(state['in_range'] for state in self.proximity_states.values()):
            self.warningLed.on()  # At least one zombie is in range, turn LED on
        else:
            self.warningLed.off()  # No zombies are in range, turn LED off

        # Clean up old entries
        for zombie_number in zombies_to_remove:
            del self.proximity_states[zombie_number]

    """
    Handles tagging logic when proximity duration is met.

    :param zombie_number: The number of the detected zombie
    """
    async def handle_tagging(self, zombie_number):
        self.tag_counts.setdefault(zombie_number, 0)
        self.tag_counts[zombie_number] += 1

        if self.verbose:
            asyncio.create_task(self.beep(0.5))
            print(f"Tagged by zombie {zombie_number}: {self.tag_counts[zombie_number]} time(s)")

        tag_data = [{'zombie_number': zn, 'tag_count': count} for zn, count in self.tag_counts.items()]
        self.write_to_file(tag_data)

        if self.tag_counts[zombie_number] >= 3:
            # Human becomes a zombie with the same number
            self.role = 'zombie'
            self.zombie_number = zombie_number
            self.is_human_game_over = True
            if self.verbose:
                print(f"Human has turned into zombie {self.zombie_number}!")

            timeTillDeath = time.time() - self.humanStartTime
            print("Lasted: ", timeTillDeath)

            with open('tag_data.txt', 'a') as f:
                f.write(f"This human lasted {timeTillDeath} seconds before being zombified! /n")
            
            self.advertiser = Yell()
            await self.run_zombie()

    """
    Stops the game by halting advertising or scanning depending on the player's role.
    """
    def stop(self):
        self.is_game_over = True
        if self.role == 'zombie':
            self.advertiser.stop_advertising()
        elif self.role == 'human':
            self.scanner.stop_scan()

zombie = Zombie()
asyncio.run(zombie.run())