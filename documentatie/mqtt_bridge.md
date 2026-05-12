# MQTT Bridge

## Wat doet het?
Stuurt de batterij status en docking state van de robot naar een MQTT broker. Andere teams kunnen zo de status van de robot live zien op hun dashboard.

## Configuratie
| Parameter | Waarde | Beschrijving |
|-----------|--------|-------------|
| ROBOT_ID | tag2 | ID van de robot (ArUco marker ID) |
| MQTT_HOST | jetson-dang.local | Adres van de MQTT broker |
| MQTT_PORT | 1883 | Poort van de broker |
| PUBLISH_INTERVAL | 5.0 sec | Hoe vaak data verstuurd wordt |

## MQTT Topics
| Topic | Inhoud | Formaat |
|-------|--------|---------|
| city/robots/tag2/battery | Batterij percentage | Integer (0-100) |
| city/robots/tag2/docking | Volledige status | JSON |

## JSON formaat
```json
{
  "state": "DOCKED",
  "charging": "OPLADEN",
  "voltage": 12.1,
  "pct": 87
}
```

## Mogelijke states
| State | Beschrijving |
|-------|-------------|
| IDLE | Robot wacht |
| SEARCH | Robot zoekt marker |
| ALIGN | Robot centreert |
| APPROACH | Robot rijdt naar station |
| DOCKED | Robot is gedockt |
| UNDOCK | Robot rijdt weg |

## Starten
```bash
# Robot met marker ID 2
source ~/ros2_ws/install/setup.bash
ROBOT_ID=tag2 ros2 run docking_robot mqtt_bridge

# Andere robot met marker ID 3
ROBOT_ID=tag3 ros2 run docking_robot mqtt_bridge
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /battery_state | sensor_msgs/BatteryState | Subscriber |

## Installatie
```bash
pip install paho-mqtt --break-system-packages
```
