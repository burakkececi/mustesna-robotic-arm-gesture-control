# 🤖 Mustesna-001 Gesture Control
Mustesna-001 project aims to develop an artificial intelligence-supported robotic arm to address occupational hazards. We develop a robotic arm that communicates with a remotely accessible platform. In the first prototype, a logic that detects hand movements and activates the robotic arm accordingly is aimed.

It uses the MediaPipe from Google to detect hand gestures and control the Mustesna's arm.

## ✒️ Design
### High Level Design
![Circuit Design](/assets/images/circuit.png "Circuit Design")
### Circuit Design
![Circuit Design](/assets/images/circuit_impl.png "Circuit Design")
### Flow Chart
![Circuit Design](/assets/images/flow.png "Circuit Design")
### Robotic Arm
![Circuit Design](/assets/images/arm.png "Circuit Design")

## ⚙️ Run
### Upload the Code to Arduino
First, change the initial position of the servo motor in the code. I used sg-90 servo motor.
Define x, y, z and claw angles.
**Make sure remove the bluetooth module from serial pin then upload the code.**
### Install the required libraries
### Set up the camera
You can use the your laptop cam or a webcam from a server.
 ```python
cam_source = "0"
# 0,1 for usbcam
 ```
### Change the configuration

Define your min-max servo movement range of your robotic arm.
Then, change the configuration in the python code.
```python
# Configurations
x_min = 13
x_mid = 93
x_max = 173

y_min = 70
y_mid = 110
y_max = 150

z_min = 113
z_mid = 146
z_max = 180

claw_open_angle = 93
claw_close_angle = 160
```

Change the debug mode on so that you dont write the serial port first.
```python
debug = True
```
Due to different camera viewing angles and resolutions, it may need test and then change the values of the following parameters.
```python
# use angle between wrist and index finger to control x axis
palm_angle_min = -50
palm_angle_mid = 20

# use wrist y to control y axis
wrist_y_min = 0.3
wrist_y_max = 0.9

# use palm size to control z axis
plam_size_min = 0.1
plam_size_max = 0.3

fist_threshold = 7

cap_width = 1400
cap_height = 900
```

### Finally, run the code and enjoy it.
Make sure the COM port is correctly defined.
```python
ser = serial.Serial('COM7', 115200)
```
Change debug mode to False.
```python
debug = False
```
Run the code.
```bash
python main.py
```
