## Welcome! We are KAU BOT

Our robot combines computer vision, sensor feedback, and intelligent control to navigate the field in both open and obstacle rounds.

Here is the YouTube video link for our robot → []
## Strategy of the Robot
# Open Round

In the open round, the robot's task is to complete three full loops around the arena without colliding with walls.

    The Raspberry Pi analyzes the live video feed from the WEB camera to keep the robot on track.\
    The ESP32 manages motor control and uses DF ultrasonic sensors to maintain safe distances from walls and corners.\
    A differential mechanism and servo steering provide smooth turns at every corner, giving the robot stable and efficient movement.

# Obstacle Round

In the obstacle round, colored cubes (green and red) are placed along the path. The Raspberry Pi processes the video, applies ROI (Region of Interest), and identifies each cube's color and distance.

    Green Cube → The Raspberry Pi commands the ESP32 to turn left around the cube.\
    Red Cube → The Raspberry Pi commands the ESP32 to turn right around the cube.

The ESP32 executes these instructions through DC motors and servo motors, while the DF ultrasonic sensors confirm safe movement.
This system allows the robot to complete its loops intelligently, reacting dynamically to obstacles.

## How to Use ESP32 as a Low-Level Controller

The ESP32 acts as the low-level controller, receiving commands from the Raspberry Pi and controlling the actuators.

    Programming: Can be programmed via Arduino IDE or PlatformIO.\
    Communication: Receives instructions from Raspberry Pi via UART and sends sensor feedback back.\
    Motor Control: Generates PWM signals for DC and servo motors.\
    Sensor Integration: Reads data from gyro and DF ultrasonic sensors.\
    Extra Features: Controls a buzzer to provide audio feedback.

## Robot Components
# Raspberry Pi (High-Level Controller)

Handles vision processing and decision-making. Runs OpenCV-based algorithms for cube detection and path planning.
Specs: Quad-core ARM Cortex-A72, up to 8GB RAM, Wi-Fi + Bluetooth, CSI camera interface.
## ESP32 (Low-Level Controller)

Executes motor commands and processes sensor feedback. Dual-core processor, built-in Wi-Fi and Bluetooth, 34 GPIO pins with ADC, PWM, I2C, SPI, UART support.
## WEB  Camera

Provides high-quality video input for real-time image analysis on the Raspberry Pi.
## Servo Motors

Used for steering and camera adjustments. Provide precise angular positioning.
## DC Motors

Drive the robot's wheels, enabling forward and backward movement.
## Differential Mechanism

Ensures smooth turning by allowing rear wheels to rotate at different speeds. Improves cornering and stability.
## DF Ultrasonic Sensors

Placed around the robot to provide 360° obstacle detection. More accurate and stable than traditional ultrasonic modules.
## Gyro Sensor

Measures angular velocity and orientation, sending feedback to the Raspberry Pi via ESP32.
## Battery

A rechargeable battery serves as the primary power source for the entire system. It is carefully selected to provide enough current to support the Raspberry Pi, ESP32, and multiple motors simultaneously. The capacity ensures that the robot can complete several rounds in competition without recharging.
## Voltage Regulator

Since the battery provides higher voltage than the sensors and controllers can handle, a voltage regulator is used to step it down to a stable 5V. This prevents electrical noise or power surges from damaging sensitive electronics such as the ESP32 and DF ultrasonic sensors.
## 3D Printing

Several mechanical components such as the body frame, servo holders, and protective covers were designed and fabricated using a 3D printer. This method allowed us to create lightweight and customized parts that fit the robot's exact dimensions, improving assembly and stability.
## Laser Cutting

A laser cutter was used to manufacture acrylic plates that serve as mounting bases for electronics. Acrylic was chosen for its rigidity and precision, ensuring that all sensors and boards remain firmly in place during high-speed ## movement.
Source Code
All source codes are available in the ⁠ src ⁠ directory.
