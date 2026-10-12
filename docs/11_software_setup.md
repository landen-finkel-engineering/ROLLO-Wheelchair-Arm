# 11 — Software setup, step by step (ESP32 + Jetson)

Commands go in a terminal exactly as written. Lines starting with `#` are comments.
Do sections A → E in order. Section F lets you try everything with no hardware.

## A. Flash the ESP32 supervisor (on your Windows PC)

**Option 1 — PlatformIO (recommended)**

1. Install **VS Code** (code.visualstudio.com). In VS Code open Extensions (Ctrl+Shift+X), search **PlatformIO IDE**, click Install, and restart VS Code.
2. *File → Open Folder…* → `Arm-Design\firmware\esp32_safety_controller`. PlatformIO reads `platformio.ini` and downloads the ESP32 toolchain the first time, which takes several minutes.
3. Plug in the ESP32 (Freenove ESP32-WROOM) with a **data** USB cable. If Windows shows no new COM port, install the CH340 driver (Freenove's tutorial package has it) and re-plug.
4. Click the **✓ Build** button in the blue status bar at the bottom. It should end with `SUCCESS` (about 285 kB of flash, 22 % of the app partition). The same code was compiled and linked here with the identical core and compiler (arduino-esp32 2.0.17, xtensa GCC 8.4.0), but never run on a board yet. If your build fails, copy the first error line and send it to me.
5. Click **→ Upload**. If it hangs at `Connecting....`, hold the board's **BOOT** button until the upload starts.
6. Click the **plug icon (Serial Monitor)**. You will see binary frames, which are not readable text; that is normal. The blue LED should blip every 2 s (SAFE_OFF).

**Option 2 — Arduino IDE 2**

1. *File → Preferences → Additional boards manager URLs* → add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
2. *Tools → Board → Boards Manager* → search **esp32** (by Espressif) → install version **2.0.17** (the version that was compiled and checked). The code also has a branch for the 3.x watchdog API, but 3.x has not been compiled.
3. *File → Open* → `firmware\esp32_safety_controller\esp32_safety_controller.ino`. The IDE also compiles the `src` folder.
4. *Tools*: Board **ESP32 Dev Module**, Upload Speed **921600**, Port = your COM port. Click **Upload**.

## B. Prepare the Jetson Orin Nano (Ubuntu 22.04, JetPack 6.x)

```bash
# 1. system update + tools
sudo apt update && sudo apt -y upgrade
sudo apt -y install git python3-pip python3-serial python3-yaml python3-scipy python3-numpy curl

# 2. ROS 2 Humble (official apt packages)
sudo apt -y install software-properties-common && sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt -y install ros-humble-ros-base ros-humble-xacro ros-humble-robot-state-publisher \
     ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc

# 3. get the project (GitHub, see README) and build the ROS 2 workspace
#    (if you make the repo private: run `sudo apt -y install gh && gh auth login` first,
#     or copy the Arm-Design folder over with a USB stick into ~/wra_ws/src/ROLLO-Wheelchair-Arm)
mkdir -p ~/wra_ws/src && cd ~/wra_ws/src
git clone https://github.com/landen-finkel-engineering/ROLLO-Wheelchair-Arm.git
# colcon finds wra_driver and wra_description inside the clone by itself (do not symlink them:
# the same package found twice stops the build with "Duplicate package names")
cd ~/wra_ws && colcon build --symlink-install --packages-select wra_driver wra_description
echo "source ~/wra_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc

# 4. serial permissions + a fixed device name for the ESP32
sudo usermod -aG dialout $USER          # log out and back in once
lsusb                                   # note the ESP32 USB-serial ID, e.g. 1a86:7523 (CH340)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="wra_esp32"' | sudo tee /etc/udev/rules.d/99-wra.rules
sudo udevadm control --reload-rules && sudo udevadm trigger   # -> /dev/wra_esp32
```

## C. First power-up and calibration (arm on the BENCH STAND, docs/09 T0–T2)

1. **E-stop test first** (docs/09 T0.2). Do not continue until pressing the E-stop cuts servo power.
2. **Give each servo its ID**, one at a time. Put the bus adapter jumper on **B (USB)** and plug its USB into the Jetson. Connect **one** new servo.
   ```bash
   cd ~/wra_ws/src/ROLLO-Wheelchair-Arm/software/tools
   python3 servo_setup.py --adapter /dev/ttyACM0 scan        # factory servo answers as ID 1
   python3 servo_setup.py --adapter /dev/ttyACM0 set-id 1 5  # e.g. this one becomes Theta1 (ID 5)
   ```
   Label each servo with its ID. The IDs are: 1 J1, 2 J2, 3 J3, 4 q1, 5 Θ1, 6 q2, 7 Θ2, 8 claw.
3. **Normal wiring.** Put the adapter jumper back to **A (UART)**, wire it to the ESP32 (docs/07), and plug the ESP32 into the Jetson.
   ```bash
   python3 wra_cli.py --port /dev/wra_esp32 status          # state SAFE_OFF, faults NOT_CALIBRATED
   python3 wra_cli.py --port /dev/wra_esp32 arm             # ARMING -> press START on the pendant (servos power up, torque off)
   python3 servo_setup.py --esp32 /dev/wra_esp32 calibrate   # follow the prompts joint by joint
   ```
   The tool asks you to place each joint in a reference pose (level, alignment marks) and then nudge it in the + direction. It sets the homing offsets, writes the servo EEPROM limits and the ESP32 limit table, and saves `config/calibration.yaml`. Then run `colcon build` again, or keep the `--symlink-install` link.
4. **Jog test** at low speed, with a hand on the E-stop. The user keep-out zone is on, because `side:` is set in calibration.yaml. On the bench stand only, `--no-keepout` lets you test the whole joint range; never use it with anyone in the chair.
   ```bash
   python3 wra_cli.py --port /dev/wra_esp32 --speed 10 arm    # READY -> ACTIVE (torque on, holds position)
   python3 wra_cli.py --port /dev/wra_esp32 --speed 10 move 0 85 -20 -20 0 0 10
   python3 wra_cli.py --port /dev/wra_esp32 --speed 10 home
   ```

## D. Run with ROS 2

```bash
ros2 launch wra_driver wra_bringup.launch.py port:=/dev/wra_esp32 side:=right rviz:=true
# in a 2nd terminal:
ros2 service call /wra/arm std_srvs/srv/SetBool "{data: true}"      # 1st call: ARMING -> press START
ros2 service call /wra/arm std_srvs/srv/SetBool "{data: true}"      # 2nd call: torque on
ros2 topic pub --once /wra/claw std_msgs/msg/Float32 "{data: 40.0}"
ros2 service call /wra/home std_srvs/srv/Trigger
ros2 topic pub --once /wra/stop std_msgs/msg/Empty                  # cancel any motion
```

**Voice (ReSpeaker).**
```bash
sudo apt -y install libportaudio2 unzip          # audio backend for sounddevice
pip3 install vosk sounddevice
cd ~ && curl -LO https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip vosk-model-small-en-us-0.15.zip
ros2 run wra_driver voice_node --ros-args -p model_path:=$HOME/vosk-model-small-en-us-0.15
```
Say "robot go home", "robot open hand", and "stop".

**Door knob.** Teach the knob once:
```bash
python3 wra_cli.py --port /dev/wra_esp32 teach-knob
```
With torque off, guide the claw onto the knob and press Enter. Then run the task:
```bash
python3 wra_cli.py --port /dev/wra_esp32 door
```
With ROS, publish `/wra/knob_pose` instead (e.g. from your stereo doorknob detector) and call `/wra/open_door`.

## E. Tests (any computer, no hardware)

```bash
cd ~/wra_ws/src/ROLLO-Wheelchair-Arm/software/wra_ros2/wra_driver
python3 -m pytest -q test          # 10 tests (~1 min): protocol, ABENICS IK/FK, arm IK, servo mapping, client vs simulated
                                   # ESP32 + E-stop, door-knob task, stiff-latch abort, Theta torque cap, user keep-out
```

## F. Simulation (no hardware at all)

```bash
python3 wra_cli.py --fake move 10 60 -15 -10 5 20 15         # simulated ESP32, prints the reached joints
ros2 run wra_driver fake_esp32 --pty                         # prints /dev/pts/N
ros2 launch wra_driver wra_bringup.launch.py port:=/dev/pts/N rviz:=true
```

## G. After you change `params.yaml` (your chair measurements, fits)

The driver takes the chair → arm transform and the user keep-out boxes from `software/wra_ros2/wra_driver/config/arm_geometry.json`. The PC writes that file; the Jetson uses the new values only after these steps.

1. **On the PC**, in the `Arm-Design` folder:
   ```bash
   cd cad
   python build.py --parts --verify     # new mount parts + clearance checks; also refreshes arm_geometry.json
   ```
   Open `output/reports/verification.md`. It must say **Overall: PASS**. Write down the 16-character code after `sha256:` in the line that starts with "Generated"; that is the `params_sha`.
2. **Copy the file to the Jetson.** With a USB stick, copy `Arm-Design\software\wra_ros2\wra_driver\config\arm_geometry.json` to `~/wra_ws/src/ROLLO-Wheelchair-Arm/software/wra_ros2/wra_driver/config/` on the Jetson and replace the old one. (If your PC copy is a git clone, you can instead commit and push, then run `git pull` in `~/wra_ws/src/ROLLO-Wheelchair-Arm` on the Jetson.)
3. **Rebuild on the Jetson:**
   ```bash
   cd ~/wra_ws && colcon build --symlink-install --packages-select wra_driver wra_description
   source ~/wra_ws/install/setup.bash
   ```
4. **Check that they match** (docs/09 test T4.0):
   ```bash
   cd ~/wra_ws/src/ROLLO-Wheelchair-Arm/software/tools
   python3 wra_cli.py --port /dev/wra_esp32 status    # "geometry_params_sha" must equal the params_sha from step 1
   ```
   Without the ESP32 connected, `python3 wra_cli.py --fake status` prints the same value. The ROS driver also logs it when it starts.
