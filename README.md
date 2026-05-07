# Docking Station voor Autonome Robots 🤖

## Wat doet dit project?
Dit project laat een robot automatisch naar een docking station rijden, opladen, en weer vertrekken.
De robot gebruikt een camera om een ArUco marker te detecteren op het docking station en stuurt zichzelf bij tot hij perfect gecentreerd is. Dan rijdt hij vooruit tot hij gedockt is.

## Hoe werkt het?
De robot doorloopt automatisch deze stappen:
- **IDLE** → wacht tot je D drukt
- **SEARCH** → geen marker? Dan draait hij rond om te zoeken
- **ALIGN** → marker gevonden! Robot stuurt bij naar links of rechts
- **APPROACH** → gecentreerd! Robot rijdt vooruit naar het station
- **DOCKED** → robot is gedockt en laadt op
- **UNDOCK** → druk U, robot rijdt achteruit en gaat terug naar IDLE

## Hardware
- Raspberry Pi 4
- 2 motoren (links/rechts)
- OpenCR motor controller
- Camera (USB of Pi Camera)
- Docking station met ArUco marker (ID 2, DICT_4X4_50)
- 12V batterij

## Software
- ROS2
- Python 3
- OpenCV (ArUco detectie)
- PySerial

## Installatie
```bash
cd ~/ros2_ws
colcon build --packages-select docking_robot
source ~/ros2_ws/install/setup.bash
```

## Hoe starten?
Je hebt 3 terminals nodig:

**Terminal 1 — Camera:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot camera_publisher
```

**Terminal 2 — Motor controller + batterij:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot serial_bridge
```

**Terminal 3 — Docking controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot docking_controller
```

## Bediening
| Toets | Actie |
|-------|-------|
| D | Start docking procedure |
| U | Undock (alleen als DOCKED) |
| CTRL+C | Stop alles |

## ROS2 Topics
| Topic | Type | Beschrijving |
|-------|------|-------------|
| /camera/image_raw | sensor_msgs/Image | Camera beelden |
| /cmd_vel | geometry_msgs/Twist | Motor commando's |
| /battery_state | sensor_msgs/BatteryState | Batterij voltage |

## Failsafes
- Marker niet gevonden na 10 sec → terug naar IDLE
- Marker kwijt tijdens rijden → stop en opnieuw zoeken
- Approach duurt te lang (30 sec) → stop
- Batterij onder 9.5V → alles stopt

## Auteurs
- Marwan
