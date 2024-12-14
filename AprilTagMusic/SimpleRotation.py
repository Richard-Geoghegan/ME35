# simpleRotation.py
# Reads April Tag ID and orientation, sends info over BLE to ESP32 for processing
# By: Anne Hu, 11/17/24

import uasyncio as asyncio
import sensor
import math
from peripheral import Peripheral

# --------- CAMERA SET UP -----------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # changing to QVGA (320x240) from QQVGA, makes it slower
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...

# --------- HELPER FUNCTIONS AND DEFINITIONS ----------
def degrees(radians):
    return (180 * radians) / math.pi


async def main(p):
    lastSent = None

    while True:
        img = sensor.snapshot()

        for tag in img.find_apriltags():  # defaults to TAG36H11
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
            angle = degrees(tag.z_rotation)
            print(angle)

            if angle > 270:
                angle = 270

            note = int(angle/34)
            print("note index: "+str(note))
            print("tag id: "+str(tag.id))

            toSend = str(note)+","+str(tag.id)

            if toSend != lastSent:
                p.send(str(toSend))
                lastSent = toSend

        # To reconnect bluetooth if it disconnects
        if not p.isConnected():
            print("Bluetooth disconnected, trying again")
            try:
                p.connect()
            except:
                print("Could not reconnect bluetooth")

        await asyncio.sleep(0.01)

# ---------------- MAIN CODE ----------------
p = Peripheral('Camera') # Creating a peripheral object named "Camera"
p.connect()
print("----- Camera is connected to BLE -------")

loop = asyncio.get_event_loop()
loop.create_task(p.messageHandler())
loop.create_task(main(p))
loop.run_forever()
