import pygame
import sys
from pygame.locals import QUIT, JOYAXISMOTION

# Initialize Pygame
pygame.init()

# Set up the screen size
screen_width, screen_height = 1920, 1080  # Running on a 1920x1080 screen
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("April Tag Control")

# Load the image (use the AprilTag image you uploaded)
image = pygame.image.load("/Users/richardgeoghegan/Documents/Robotics/tag36h11_8.png")
original_image_rect = image.get_rect()

# Tag should begin fully visible on the screen
initial_scale = 0.4  # Start smaller so it's fully visible initially
scale_factor = initial_scale

# Max scale based on keeping the tag fully visible at max size (approx 75% of the screen height)
max_scale = min(screen_width / original_image_rect.width, screen_height / original_image_rect.height) * 0.75
min_scale = 0.2  # The smallest size it can go
default_scale = initial_scale  # Reset point

# Rotation limits
angle = 0
default_angle = 0  # Starting angle
max_angle = 30  # Max rotation in degrees

# Adjust sensitivity for smoother, less sensitive control
rotation_speed = 0.3  # Slower rotation
scale_speed = 0.001  # Slower scaling for smoother movement
return_speed = 0.1  # Slower return to default state

# Input smoothing
smooth_factor = 0.05  # Smoothing factor for gradual movement
left_stick_y_prev = 0  # Left stick up/down (axis 1) for scaling
right_stick_x_prev = 0  # Right stick left/right (axis 2) for rotation

# Initialize joystick
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    print("No joystick detected!")
    sys.exit()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

        # Handle joystick motion
        if event.type == JOYAXISMOTION:
            # Left stick vertical controls scaling (axis 1)
            left_stick_y = joystick.get_axis(1)  # Up/down for scaling
            left_stick_y_smooth = left_stick_y_prev + smooth_factor * (left_stick_y - left_stick_y_prev)

            # Apply scaling, ensuring it remains within the bounds
            if left_stick_y_smooth < -0.1 and scale_factor < max_scale:
                scale_factor += scale_speed  # Moving up increases size
            elif left_stick_y_smooth > 0.1 and scale_factor > min_scale:
                scale_factor -= scale_speed  # Moving down decreases size
            left_stick_y_prev = left_stick_y_smooth

            # Right stick horizontal controls rotation (axis 2)
            right_stick_x = joystick.get_axis(2)  # Left/right for rotation
            right_stick_x_smooth = right_stick_x_prev + smooth_factor * (right_stick_x - right_stick_x_prev)

            # Apply rotation when the stick is moved
            if right_stick_x_smooth < -0.1:
                angle = min(angle + rotation_speed, max_angle)  # Left rotates clockwise
            elif right_stick_x_smooth > 0.1:
                angle = max(angle - rotation_speed, -max_angle)  # Right rotates counterclockwise
            right_stick_x_prev = right_stick_x_smooth

            # If both sticks are near the neutral position, return the image to its default state
            if abs(left_stick_y) < 0.1:  # Stick is neutral for scaling
                if scale_factor > default_scale:
                    scale_factor = max(scale_factor - return_speed, default_scale)  # Gradually reduce the size
                elif scale_factor < default_scale:
                    scale_factor = min(scale_factor + return_speed, default_scale)  # Gradually increase the size

            if abs(right_stick_x) < 0.1:  # Stick is neutral for rotation
                if angle > default_angle:
                    angle = max(angle - return_speed * 10, default_angle)  # Gradually rotate back to neutral
                elif angle < default_angle:
                    angle = min(angle + return_speed * 10, default_angle)  # Gradually rotate back to neutral

    # Clear the screen
    screen.fill((255, 255, 255))

    # Transform the image (scale and rotate)
    rotated_image = pygame.transform.rotate(image, angle)
    scaled_image = pygame.transform.scale(rotated_image, (int(original_image_rect.width * scale_factor), int(original_image_rect.height * scale_factor)))
    scaled_rect = scaled_image.get_rect(center=(screen_width // 2, screen_height // 2))

    # Draw the image
    screen.blit(scaled_image, scaled_rect.topleft)

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
sys.exit()
