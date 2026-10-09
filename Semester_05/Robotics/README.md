# Robotics

Semester 5 · Year 3 · Python, ROS 1, OpenCV

Programming a Hiwonder PuppyPi robot dog with ROS: walking gaits, reading the camera, detecting colored objects, and closing the loop so the robot tracks or kicks a ball.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | `pupy_demo.py`: sets pose, gait (Trot/Amble/Walk) and velocity through `puppy_control` topics |
| [Labs/Lab_04](Labs/Lab_04) | Camera color detection: color threshold, largest contour, draws a hexagon around it (`main.py`). `aa` is a second version of the same idea without a file extension |
| [Labs/Lab_05](Labs/Lab_05) | Course files. `curs5_track.py` and `main.py` are copies of the Lab_04 scripts. `curs6.py`: color tracking with SEARCHING/APPROACH/OPTIMAL states that tilts the head and walks to the object. `curs7.py`: turns and walks toward a detected object. `lab6.py`: gait demo with an extra Gallop gait |
| [Labs/Lab_06](Labs/Lab_06) | `main.py`: detects a colored ball, centers it, walks up to it and runs a kick action. `kick_ball_demo.py` is Hiwonder's official kick-ball demo used as reference |

## How to run

These scripts run on the robot (or a machine on the robot's ROS network) where the Hiwonder packages are installed:

```sh
python3 Labs/Lab_06/main.py
```

They subscribe to `/usb_cam/image_raw` and publish to `/puppy_control/*`.

## Notes

- Needs the PuppyPi hardware and its ROS image. The `puppy_control`, `sensor`, `ros_robot_controller` and `object_tracking` message packages come with the robot and are not in this repo.
- Color thresholds are hardcoded for red in LAB space. Recalibrate for your lighting.
- Comments in `pupy_demo.py`, `lab6.py` and `kick_ball_demo.py` are in Chinese (copied from Hiwonder samples).
- `Labs/Lab_04/requirements.txt` only pins `opencv-python`.
