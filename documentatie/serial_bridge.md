# Serial Bridge

## Wat doet het?
Brug tussen ROS2 en de OpenCR motor controller. Ontvangt /cmd_vel commando's en stuurt ze via serial naar de motoren. Leest ook de batterij spanning uit en publiceert die op /battery_state.

## Hardware
| Parameter | Waarde |
|-----------|--------|
| Poort | /dev/ttyACM0 |
| Baudrate | 57600 |
| Board | OpenCR |

## Motor commando formaat
```
D {links} {rechts} 1
```
- Links en rechts zijn PWM waarden tussen -80 en 80
- Positief = vooruit, negatief = achteruit

## Voorbeelden
| Commando | Actie |
|----------|-------|
| D 14 14 1 | Vooruit |
| D -15 -15 1 | Achteruit |
| D -10 10 1 | Draai links |
| D 10 -10 1 | Draai rechts |
| D 0 0 1 | Stop |

## Batterij uitlezen
Stuurt `battery` commando naar OpenCR.
OpenCR antwoordt met `BatteryVoltage:11.78 V`.
Wordt elke 5 seconden uitgelezen.

## Parameters
| Parameter | Waarde | Beschrijving |
|-----------|--------|-------------|
| max_pwm | 80 | Maximum PWM waarde |
| linear_gain | 50.0 | Gain voor vooruit/achteruit |
| angular_gain | 50.0 | Gain voor draaien |

## Starten
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot serial_bridge
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /cmd_vel | geometry_msgs/Twist | Subscriber |
| /battery_state | sensor_msgs/BatteryState | Publisher |
