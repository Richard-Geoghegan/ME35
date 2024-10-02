# The Mystery Minnow

### By Richard Geoghegan and Julian Moody

Welcome to the Mystery Minnow project! 🚐 Our goal was to create a one-of-a-kind Scooby Doo Mystery Machine, controlled by an AprilTag and guided through treacherous waters filled with enemy sharks (or in our case, the SEC!). The Mystery Machine is equipped with Raspberry Pi Picos, DC motors, and an OpenMV camera to create an interactive and thrilling experience.

### Project Overview

The Mystery Minnow project consists of two parts:

1. **The Mystery Machine** – A motorized vehicle driven by two Raspberry Pi Picos and controlled via an MQTT system.
2. **The OpenMV Camera Mount** – A mounted camera setup that detects the AprilTag on the controlling device and adjusts motor speeds accordingly.

### How It Works

- **Motors and Picos**: Each Raspberry Pi Pico controls a DC motor using PWM and listens for MQTT messages to adjust motor speed. The motor speed is determined by the AprilTag size and angle, as captured by the OpenMV camera.
  
- **Steering Mechanism**: A Nintendo Switch controller adjusts the AprilTag size and angle. Moving the left joystick controls forward/reverse movement by increasing or decreasing the tag size. The right joystick adjusts the angle for left or right turns. Watch it in action [here](https://youtu.be/yywP8cnmCqU).

### MQTT and Teachable Machine Integration

The Mystery Machine can also be started and stopped via a Teachable Machines model. With a simple thumbs up 👍🏼 or palm ✋🏼 gesture, you can control the motors remotely. Check out the demo [here](https://youtu.be/N9wVXPvfcjw).

### Code Snippets

The project consists of four main Python programs:

- `control.py`: Handles the joystick input and adjusts the AprilTag on the screen.
- `teachableMachines.py`: Uses a Teachable Machines model to start and stop the car via MQTT.
- `main.py`: Runs on each Pico, controls the motor, and subscribes to MQTT.
- `camera.py`: Runs on the OpenMV camera, detects AprilTag size and angle, and sends motor speed commands via MQTT.

### Results

The Mystery Machine successfully navigated a sea of sharks (aka the SEC) and reached safety. You can see the journey in action [here](https://youtu.be/U2Pm0H5OdSs). The car's performance exceeded our expectations, providing sharp and responsive turns to avoid obstacles.

### Lessons Learned

- Motors require a minimum speed to turn effectively. We found that 5% speed is not enough for movement.
- Calculating left and right motor speeds for turns was a challenge but ultimately led to satisfying results with sharp turns!

For more details, check out the full project documentation [here](https://github.com/Richard-Geoghegan/ME35/blob/main/SharksandMinnows/README.md).

### Jinkies! Future Improvements 🛠

- Adding more sensors for better obstacle detection.
- Fine-tuning the control algorithm for smoother turns and speed adjustments.

Feel free to explore and contribute to the Mystery Minnow project! 🎉
