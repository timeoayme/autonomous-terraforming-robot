## Integrated Autonomous Robot Controller

The integrated controller combines:

* Black-line navigation
* IMU-guided turning
* RFID checkpoint handling
* MiniMessenger server communication
* Ultrasonic obstacle detection
* Tunnel wall following
* Automated seed planting
* Emergency return-to-base behaviour

The codebase is developed as a **PlatformIO** project. The public GitHub repository also contains exported top-level source files, system diagrams, flowcharts, test logs, screenshots, and field-test videos for assessment and review.

## Repository Structure

```text
.
├── final.cpp
├── planting.cpp
├── return_to_base.cpp
├── platform_io/
├── secrets.h
├── software_overview_diagram.png
├── flowcharts/
│   ├── mechanical_kill_switch.png
│   ├── RFID_planting.png
│   └── line_following.png
└── Testing_Calibration_Evidence/
```

| File or directory               | Description                                                   |
| ------------------------------- | ------------------------------------------------------------- |
| `final.cpp`                     | Main integrated robot controller                              |
| `planting.cpp`                  | Standalone field-planting controller                          |
| `return_to_base.cpp`            | Standalone emergency return test                              |
| `platform_io/`                  | PlatformIO project configuration and exported project files   |
| `secrets.h`                     | Private network and server configuration                      |
| `software_overview_diagram.png` | High-level software architecture diagram                      |
| `flowcharts/`                   | Flowcharts for major robot behaviours                         |
| `Testing_Calibration_Evidence/` | Logs, screenshots, calibration records, and field-test videos |

## Main Controller Behaviour

The integrated controller:

* Starts in a stopped safety state and waits for the mechanical **D45 run toggle**.
* Registers with the challenge server through MiniMessenger when online mode is enabled.
* Continuously services:

  * server messages;
  * control buttons;
  * status LEDs;
  * motor encoders;
  * IMU yaw data;
  * RFID input;
  * planting servo control;
  * ultrasonic sensors.
* Follows a black line using five reflectance sensor outputs and PID steering.
* Detects corners and T-junctions, then performs IMU-guided 90° or 180° turns.
* Uses short search turns, yaw-hold driving, or tunnel wall following when the line is lost.
* Treats the first detected RFID tag as the airlock checkpoint.
* Treats later RFID tags as fertility-query checkpoints.
* Plants a seed only after receiving a positive fertility response from the server.
* Queues a `seedPlanted` report after the servo planting cycle completes.
* Stops the current action during an emergency and plans a return route towards grid exit `(9, 7)`.

## Software Architecture

The main controller is designed around a continuously running service loop.

```text
Start
  │
  ▼
Safety wait for D45
  │
  ▼
Initialise hardware and communications
  │
  ▼
Main control loop
  ├── Process MiniMessenger messages
  ├── Read safety buttons
  ├── Update LEDs
  ├── Read reflectance sensors
  ├── Update motor encoders
  ├── Update IMU yaw
  ├── Check RFID
  ├── Check ultrasonic sensors
  ├── Run navigation behaviour
  ├── Run planting behaviour
  └── Handle emergency return
```

The main loop must continue calling:

```cpp
serverMessenger.loop();
```

This ensures that emergency, disable, heartbeat, airlock, and RFID fertility messages are processed promptly.

## Required Libraries

PlatformIO installs the external dependencies through `lib_deps`.

| Library                                                    | Purpose                                               |
| ---------------------------------------------------------- | ----------------------------------------------------- |
| `Motoron`                                                  | Dual motor-controller communication and motor control |
| `MFRC522_I2C`                                              | WS1850S / MFRC522-compatible RFID communication       |
| `SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library` | Orientation and yaw measurements                      |
| `ArduinoMqttClient`                                        | MQTT communication                                    |
| `MiniMessenger`                                            | Challenge-server message handling                     |
| `Servo`                                                    | Seed-planting servo control                           |
| `Arduino`                                                  | Core Arduino framework                                |
| `Wire`                                                     | I²C communication                                     |

## Private Configuration

Keep `secrets.h` in the same source or sketch directory as the integrated controller.

Do not publish real credentials. Use a local file similar to:

```cpp
#pragma once

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define BROKER_HOST "YOUR_BROKER_IP"
#define BROKER_PORT 1883
#define GROUP_ID "YOUR_GROUP_ID"
#define BOARD_ID "YOUR_BOARD_ID"
```


## Build and Upload

### Compile the project

Run a full compilation before submitting changes:

```bash
pio run
```

### Upload the default integrated controller

```bash
pio run -t upload
```

### Upload a specific PlatformIO environment

```bash
pio run -e final -t upload
```


### Open the serial monitor

```bash
pio device monitor \
  -e giga_r1_m7 \
  --port /dev/cu.usbmodem101 \
  --baud 115200
```

The serial port may differ between computers. To list available devices, run:

```bash
pio device list
```

### PlatformIO executable not on `PATH`

On macOS, the PlatformIO executable may be located inside its virtual environment:

```bash
~/.platformio/penv/bin/platformio run
```

To upload:

```bash
~/.platformio/penv/bin/platformio run -t upload
```

Avoid documenting a machine-specific username such as `/Users/admin/` because that path will not work on other computers.

## Arduino IDE Export

PlatformIO is the preferred development environment.

For an Arduino IDE submission:

1. Copy the validated integrated source into an Arduino sketch such as `final.ino`.

2. Place a private `secrets.h` tab in the same sketch folder.

3. Install all required libraries.

4. Select:

   ```text
   Arduino Mbed OS GIGA Boards → Arduino GIGA R1
   ```

5. Verify the sketch.

6. Upload it to the board.

7. Open the Serial Monitor at `115200` baud.

## Operating Checklist

Before testing the robot:

1. Place the robot on a safe test surface.
2. Keep the driven wheels clear of the ground until movement is expected.
3. Connect the USB-C cable.
4. Open the Serial Monitor at `115200` baud.
5. Keep the robot stationary during IMU gyroscope and tilt calibration.
6. Confirm successful startup messages for:

   * the IMU;
   * Motoron controllers at `0x10` and `0x11`;
   * the RFID module at `0x28`;
   * the ultrasonic sensors;
   * reflectance sensors and motor encoders.
7. Confirm that the red startup LED indicates the stopped safety state.
8. Press **D45** once to permit movement.
9. Use **D45** to toggle between stopped and running states during local testing.
10. Use **D32** for revive or green-status indication after manual intervention.
11. In online mode, keep the laptop or debugging device connected to the laboratory network.
12. Verify server-dashboard enable and disable behaviour before beginning a full run.

## Server Communication

### Outgoing Messages

The robot may send the following message types:

```text
type=register
type=openAirlock
type=isFertile
type=seedPlanted
```

| Message       | Purpose                                           |
| ------------- | ------------------------------------------------- |
| `register`    | Registers the robot with the challenge server     |
| `openAirlock` | Requests access through the airlock checkpoint    |
| `isFertile`   | Queries whether the current RFID field is fertile |
| `seedPlanted` | Reports completion of a planting action           |

### Incoming Messages

The controller handles:

```text
type=heartbeat enable=...
type=disable enabled=...
type=emergency enabled=...
type=openAirlockReply accepted=...
type=isFertileReply fertile=... planted=... x=... y=...
```

| Message            | Purpose                                                       |
| ------------------ | ------------------------------------------------------------- |
| `heartbeat`        | Communicates the current server-enabled state                 |
| `disable`          | Enables or disables robot operation remotely                  |
| `emergency`        | Activates or clears emergency behaviour                       |
| `openAirlockReply` | Reports whether the airlock request was accepted              |
| `isFertileReply`   | Returns field fertility, planting state, and grid coordinates |

## Navigation Behaviour

### Line Following

The robot follows the black line using five reflectance sensor outputs and PID steering.

The controller uses the estimated line position to calculate a steering correction:

```text
error = target position - measured line position
```

The PID output adjusts the relative motor speeds to keep the robot centred over the line.

### Junction Handling

The navigation controller detects:

* corners;
* left and right turns;
* T-junctions;
* dead ends;
* line-loss conditions.

Detected junctions are resolved using IMU-guided turns, typically through:

* 90° left turns;
* 90° right turns;
* 180° U-turns.

### Line-Loss Recovery

When the line is lost, the controller may use:

1. A short directional search turn.
2. IMU yaw-hold driving.
3. Ultrasonic tunnel wall following.
4. A recovery turn based on the most recently detected line direction.

## RFID and Planting Behaviour

The first recognised RFID checkpoint is treated as the airlock.

Later RFID tags initiate a field-fertility query:

```text
Robot detects RFID
        │
        ▼
Send isFertile request
        │
        ▼
Wait for isFertileReply
        │
        ├── fertile=false ──► Continue navigation
        │
        ├── planted=true ───► Continue navigation
        │
        └── fertile=true
                │
                ▼
         Run servo planting cycle
                │
                ▼
         Send seedPlanted report
```

The robot must not plant until a positive server reply confirms that:

* the field is fertile; and
* the field has not already been planted.

## Emergency Behaviour

When an emergency command is received, the controller:

1. Stops the current navigation or planting action.
2. Prevents normal mission behaviour from continuing.
3. Determines the current grid position and orientation.
4. Plans a route towards exit coordinate `(9, 7)`.
5. Executes the return-to-base behaviour while continuing to process safety and server messages.

Emergency behaviour should always have priority over normal line following, RFID handling, and planting.

## Testing and Calibration Evidence

Compile the project before recording test evidence:

```bash
pio run
```

Capture serial output using:

```bash
pio device monitor \
  -e giga_r1_m7 \
  --port /dev/cu.usbmodem101 \
  --baud 115200
```

Recommended evidence includes:

* Successful PlatformIO compilation
* Complete startup log
* IMU initialisation and calibration
* Motoron detection at addresses `0x10` and `0x11`
* RFID initialisation at address `0x28`
* Ultrasonic sensor initialisation
* Reflectance sensor readings on:

  * black line;
  * light floor;
  * junctions;
  * line-loss conditions.
* PID line-following tests
* 90° corner-turn logs
* 180° U-turn logs
* RFID airlock interaction
* Fertility-query response
* Successful planting cycle
* `seedPlanted` transmission
* Remote disable behaviour
* Emergency return-to-base behaviour
* Short videos showing:

  * stable line following;
  * junction handling;
  * U-turns;
  * slope traversal;
  * tunnel wall following;
  * RFID detection;
  * seed planting.

## Known Limitations

* Robot performance depends on reflectance and ultrasonic calibration for the test environment.
* IMU drift may reduce turning accuracy over long runs.
* RFID and server replies introduce asynchronous delays.
* Network loss may prevent online checkpoint handling.
* Line-loss recovery depends on the robot's previous direction and surrounding geometry.
* Emergency route planning assumes that the robot's internal grid position remains accurate.
