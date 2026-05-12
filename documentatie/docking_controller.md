# Docking Controller

## Wat doet het?
De hoofdnode die de volledige docking procedure beheert. Hij gebruikt de camera om een ArUco marker te detecteren, centreert de robot, rijdt naar het docking station en detecteert wanneer de robot aan het opladen is via de batterij spanning.

## State Machine
| State | Beschrijving |
|-------|-------------|
| IDLE | Wacht op D toets |
| SEARCH | Draait rond om ArUco marker te zoeken |
| ALIGN | Centreert de robot op de marker |
| APPROACH | Rijdt rechtdoor naar het station |
| DOCKED | Robot is gedockt en laadt op |
| UNDOCK | Rijdt achteruit en draait 180 graden |

## Parameters
| Parameter | Waarde | Beschrijving |
|-----------|--------|-------------|
| center_threshold | 10px | Maximale fout voor gecentreerd |
| forward_speed | 0.28 | Snelheid vooruit |
| max_turn | 0.1 | Maximale draaisnelheid |
| wait_after_turn | 1.5sec | Wachttijd na draai voor meting |
| undock_back_time | 20sec | Tijd achteruit rijden |
| undock_turn_time | 16sec | Tijd voor 180 graden draaien |
| approach_timeout | 60sec | Failsafe timeout approach |
| search_timeout | 15sec | Failsafe timeout search |

## Bediening
| Toets | Actie |
|-------|-------|
| D | Start docking procedure |
| U | Undock (altijd mogelijk vanuit elke state) |
| CTRL+C | Stop alles onmiddellijk |

## Starten
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot docking_controller
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /camera/image_raw | sensor_msgs/Image | Subscriber |
| /battery_state | sensor_msgs/BatteryState | Subscriber |
| /cmd_vel | geometry_msgs/Twist | Publisher |

## Failsafes
- Marker niet gevonden na 15sec → IDLE
- Marker kwijt tijdens rijden → terug naar SEARCH
- Approach duurt langer dan 60sec → IDLE
- Batterij onder 9.5V → alles stopt
- Spanning stijgt tijdens approach → DOCKED
