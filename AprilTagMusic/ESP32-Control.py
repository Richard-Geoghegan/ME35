from machine import Pin, PWM
import time
import uasyncio as asyncio
from neopixel import NeoPixel
from BLE_CEEO import Listen
# from networking import Networking
from now import Now

class AprilTagMusicController:

    # Pin configurations and constants
    RESET_PIN = 17 # D7
    ADD_PIN   = 19 # D8
    PLAY_PIN  = 20 # D9
    BLUETOOTH_DEVICE_NAME = 'Camera' # OpenMV Peripheral name

    SHUTDOWN_PIN = 16 # D6
    AUDIO_PIN = 23 # D5
    SPEAKER_PWM = 50 # The duty cycle to change volume

    # Indicator pins
    BLE_INDICATOR = 1 # D1
    NOW_INDICATOR = 2 # D2

    # NeoPixel configuration
    DATA_PIN = 18   # GPIO pin connected to NeoPixel data in
    WIDTH = 32      # Number of columns
    HEIGHT = 8      # Number of rows
    NUM_PIXELS = WIDTH * HEIGHT
    BRIGHTNESS_MULTIPLIER = 0.5 # 0 is 0% brightness, 1 is 100% brightness

    # Define colors for octaves
    OCTAVE_COLORS = {
        0: (255, 255, 0),  # Yellow
        1: (255, 0, 0),    # Red
        2: (0, 0, 255),    # Blue
    }

    # All possible note frequencies (Hz)
    belowC = [131, 147, 165, 175, 196, 220, 247, 262]   # Octave 0
    middleC = [262, 294, 330, 349, 392, 440, 494, 523]  # Octave 1
    aboveC = [523, 587, 659, 698, 784, 880, 988, 1047]  # Octave 2
    Notes = [belowC, middleC, aboveC]

    def __init__(self):
        # Initialize buttons
        self.resetButton = Pin(self.RESET_PIN, Pin.IN, Pin.PULL_UP)
        self.addButton   = Pin(self.ADD_PIN, Pin.IN, Pin.PULL_UP)
        self.playButton  = Pin(self.PLAY_PIN, Pin.IN, Pin.PULL_UP)

        # Initialize indicators
        self.bleIndicator = Pin(self.BLE_INDICATOR, Pin.OUT)
        self.nowIndicator = Pin(self.NOW_INDICATOR, Pin.OUT)
        self.bleIndicator.value(0)
        self.nowIndicator.value(0)
        self.msgIn = False
        self.msgOut = False

        self.prev_reset_state = False
        self.prev_add_state   = False
        self.prev_play_state  = False
        self.playingSong = False
        self.playCurrent = False  # Flag to play the current note being read by the tag

        # Initialize the speaker
        self.shutdownPin = Pin(self.SHUTDOWN_PIN, Pin.OUT)
        self.shutdownPin.value(1)
        self.speaker = PWM(Pin(self.AUDIO_PIN, Pin.OUT))
        self.speaker.duty(0)

        # Active note and octave
        self.activeNoteIndex = 0  # Received over Bluetooth from OpenMV cam (row)
        self.activeFoodIndex = 0  # Octave (0, 1, 2)
        self.NotesQueue = []      # Contains the current notes

        # self.networking = Networking()
        self.n = Now()
        self.n.connect()

        # Initialize NeoPixel
        self.np = NeoPixel(Pin(self.DATA_PIN), self.NUM_PIXELS)
        self.current_column = self.WIDTH - 1  # Start at the rightmost column
        self.notes = []  # List to keep track of notes for NeoPixel display

        self.animating = False  # Flag to indicate if an animation is in progress

    # Helper function to map (row, column) to NeoPixel index for snake-like wiring
    def get_pixel_index(self, row, column):
        if column % 2 == 0:
            # Even columns are wired bottom-to-top
            return (column * self.HEIGHT) + row
        else:
            # Odd columns are wired top-to-bottom
            return (column * self.HEIGHT) + (self.HEIGHT - 1 - row)

    # Function to clear all pixels
    def clear_pixels(self):
        for i in range(self.NUM_PIXELS):
            self.np[i] = (0, 0, 0)
        self.np.write()

    # Function to adjust brightness
    def adjust_brightness(self, color):
        return tuple(int(c * self.BRIGHTNESS_MULTIPLIER) for c in color)

    # Wheel function to generate rainbow colors
    def wheel(self, pos):
        pos = pos % 256
        if pos < 85:
            return (int(pos * 3), int(255 - pos * 3), 0)
        elif pos < 170:
            pos -= 85
            return (int(255 - pos * 3), 0, int(pos * 3))
        else:
            pos -= 170
            return (0, int(pos * 3), int(255 - pos * 3))

    # Function to restore all notes to their octave colors after animation
    def restore_notes(self):
        for note in self.notes:
            index = self.get_pixel_index(note['row'], note['column'])
            color = self.OCTAVE_COLORS.get(note['octave'], (255, 255, 255))  # Default to white if octave invalid
            self.np[index] = self.adjust_brightness(color)
        self.np.write()

    # Function to animate a single note on NeoPixel
    async def animate_note(self, note):
        column = note['column']
        row = note['row']

        # Duration of the animation
        duration = 0.5  # 0.5 seconds
        start_time = time.ticks_ms()
        elapsed = 0

        # Save current state of the column to restore later
        original_colors = [self.np[self.get_pixel_index(r, column)] for r in range(self.HEIGHT)]

        while elapsed < duration * 1000:
            for r in range(row + 1):
                index = self.get_pixel_index(r, column)
                # Generate rainbow color
                color = self.wheel((r * 256 // self.HEIGHT + (time.ticks_ms() // 10)) % 256)
                color = self.adjust_brightness(color)
                self.np[index] = color
            self.np.write()
            await asyncio.sleep(0.05)
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)

        # Restore the original colors of the column
        for r in range(self.HEIGHT):
            index = self.get_pixel_index(r, column)
            self.np[index] = original_colors[r]
        self.np.write()

    # Function to perform the wiping animation and clear all notes
    async def stop_animation(self):
        print("Stopping and clearing all notes...")
        wait_ms = 0.05  # Duration between frames in seconds

        # Iterate over columns from left to right to create a 1-column-thick wiping effect
        for column in range(self.WIDTH):
            # Clear the previous column
            if column > 0:
                previous_column = column - 1
                for row in range(self.HEIGHT):
                    index = self.get_pixel_index(row, previous_column)
                    self.np[index] = (0, 0, 0)

            # Light up current column in red
            for row in range(self.HEIGHT):
                index = self.get_pixel_index(row, column)
                self.np[index] = self.adjust_brightness((255, 0, 0))
            self.np.write()
            await asyncio.sleep(wait_ms)

        # Clear the last column after the wipe
        for row in range(self.HEIGHT):
            index = self.get_pixel_index(row, self.WIDTH - 1)
            self.np[index] = (0, 0, 0)
        self.np.write()

        # Clear the notes list
        self.notes.clear()
        self.NotesQueue.clear()
        self.current_column = self.WIDTH - 1  # Reset current_column
        print("All notes have been cleared.")

    # Function to flash all notes red once
    async def flash_notes_red(self):
        print("Maximum notes reached! Flashing all notes red.")
        # Save current colors of note pixels
        original_colors = {}
        for note in self.notes:
            index = self.get_pixel_index(note['row'], note['column'])
            original_colors[index] = self.np[index]
            # Change to red
            self.np[index] = self.adjust_brightness((255, 0, 0))
        self.np.write()
        await asyncio.sleep(0.5)  # Flash duration

        # Restore original colors based on octave
        self.restore_notes()

    # Plays the current note received via Bluetooth
    async def playCurrentNote(self):
        while True:
            if not self.playingSong and self.playCurrent:
                print(f"Playing current note: {self.activeNoteIndex}, Octave: {self.activeFoodIndex}")
                # Play the note
                row = self.activeNoteIndex
                octave = int(self.activeFoodIndex)
                if 0 <= row < self.HEIGHT and 0 <= octave <= 2:
                    Notes = self.Notes[octave]
                    self.speaker.freq(Notes[row])
                    self.speaker.duty(self.SPEAKER_PWM)
                    await asyncio.sleep(0.5)  # Play note for half a second
                    self.speaker.duty(0)
                    await asyncio.sleep(0.5)  # Wait for half a second
                else:
                    print("Invalid note or octave.")
                self.playCurrent = False  # Reset the flag
            await asyncio.sleep(0.01)

    # Adds a note to the play queue and updates the NeoPixel array
    async def addNote(self):
        if self.animating:
            return  # Ignore button presses during animations
        if len(self.notes) >= self.WIDTH:
            await self.flash_notes_red()
            return

        print(f"Adding note: {self.activeNoteIndex}, Octave: {self.activeFoodIndex}")
        # Add the note to the queue
        octave = int(self.activeFoodIndex)
        row = self.activeNoteIndex
        if 0 <= row < self.HEIGHT and 0 <= octave <= 2:
            Notes = self.Notes[octave]
            self.NotesQueue.append(Notes[row])
            # Update the NeoPixel array
            index = self.get_pixel_index(row, self.current_column)
            color = self.OCTAVE_COLORS.get(octave, (255, 255, 255))
            self.np[index] = self.adjust_brightness(color)
            self.np.write()
            # Add note to the list with its position and octave
            self.notes.append({'row': row, 'column': self.current_column, 'octave': octave})
            # Move to the next column (right to left)
            self.current_column = (self.current_column - 1) % self.WIDTH
        else:
            print("Invalid note or octave.")

    # Removes all notes from the queue and performs the stop animation
    async def resetNotes(self):
        if self.animating:
            return  # Ignore button presses during animations
        self.animating = True
        await self.stop_animation()
        self.animating = False

    # Plays all the notes stored in the queue and animates the NeoPixel array
    async def playNotes(self):
        if self.animating:
            return  # Ignore button presses during animations
        print("Playing notes!")
        if not self.NotesQueue:
            print("No notes to play.")
            return
        self.animating = True
        self.playingSong = True
        for i, Note in enumerate(self.NotesQueue):
            # Animate the note on the NeoPixel array
            note_info = self.notes[i]
            await self.animate_note(note_info)
            # Play the note
            self.speaker.freq(Note)
            self.speaker.duty(self.SPEAKER_PWM)
            await asyncio.sleep(0.5)  # Play note for half a second
            self.speaker.duty(0)
            # await asyncio.sleep(0.1)  # Wait for half a second
        self.playingSong = False
        # After playback, restore the notes
        self.restore_notes()
        self.animating = False
        print("Playback complete.")

    # Catches Bluetooth messages, splits into note index and octave
    async def getActiveNoteBluetooth(self):
        listen_device = Listen(name=self.BLUETOOTH_DEVICE_NAME, verbose=True)
        while True:
            try:
                if listen_device.connect_up(timeout=5000):
                    print(f"Connected to {self.BLUETOOTH_DEVICE_NAME}")
                    self.bleIndicator.value(1)
                    while True:
                        if listen_device.is_any:
                            # Read and update self.activeNoteIndex with the received data
                            received_message = listen_device.read()
                            print(f"Received message: {received_message}")
                            self.msgIn = True

                            try:
                                indexes = received_message.split(',')
                                if len(indexes) >= 2:
                                    self.activeNoteIndex = int(indexes[0].strip())
                                    self.playCurrent = True  # Flag to play the current note

                                    if self.activeFoodIndex != indexes[1]:
                                        foodIndex = indexes[1]
                                        if len(foodIndex) > 1:
                                            foodIndex = foodIndex[0]
                                        self.activeFoodIndex = foodIndex
                                        print(f"Updated activeFoodIndex: {self.activeFoodIndex}")
                                        await self.sendFoodIndex()
                                else:
                                    print("Received message does not contain a valid pair of numbers")
                            except ValueError:
                                print(f"Invalid data received: {received_message}")
                        await asyncio.sleep(0)
                else:
                    print(f"Failed to connect to {self.BLUETOOTH_DEVICE_NAME}")
                    self.bleIndicator.value(0)
                    await asyncio.sleep(1)
            except Exception as e:
                self.bleIndicator.value(0)
                print(f"An error occurred: {e}")
                listen_device.disconnect()
                print("Attempting to reconnect...")
                await asyncio.sleep(1)

    async def sendFoodIndex(self):
        # recipient_mac = b'\xff\xff\xff\xff\xff\xff'
        # self.networking.aen.send(recipient_mac, self.activeFoodIndex)
        self.n.publish(str(self.activeFoodIndex).encode())
        self.msgOut = True

    async def checkButtons(self):
        while True:
            if not self.animating:
                current_reset_state = self.resetButton.value()
                if self.prev_reset_state and not current_reset_state:
                    print('-------- Reset button pressed! --------')
                    await self.resetNotes()

                current_add_state = self.addButton.value()
                if self.prev_add_state and not current_add_state:
                    print('--------- Add button pressed! ---------')
                    await self.addNote()

                current_play_state = self.playButton.value()
                if self.prev_play_state and not current_play_state:
                    print('-------- Play button pressed! ---------')
                    await self.playNotes()

                # Update previous button states
                self.prev_reset_state = current_reset_state
                self.prev_add_state = current_add_state
                self.prev_play_state = current_play_state

            await asyncio.sleep(0.1)  # Small delay to debounce

    async def checkIndicators(self):
        while True:
            if self.msgIn:
                for _ in range(2):
                    self.bleIndicator.value(0)
                    await asyncio.sleep(1)
                    self.bleIndicator.value(1)
                    await asyncio.sleep(1)
                self.msgIn = False
            if self.msgOut:
                for _ in range(2):
                    self.nowIndicator.value(0)
                    await asyncio.sleep(1)
                    self.nowIndicator.value(1)
                    await asyncio.sleep(1)
                self.msgOut = False
            await asyncio.sleep(0.01)

    async def main(self):
        await asyncio.gather(
            self.getActiveNoteBluetooth(),
            self.checkButtons(),
            self.playCurrentNote(),
            self.checkIndicators()
        )

# Create an instance of the controller class and run the code
controller = AprilTagMusicController()
asyncio.run(controller.main())
