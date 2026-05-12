# MQTT Bridge

## Wat doet het?
Stuurt de batterij status en docking state van de robot naar een MQTT broker. Andere teams kunnen zo de status van de robot live zien op hun dashboard.

## Configuratie
| Parameter | Waarde | Beschrijving |
|-----------|--------|-------------|
| ROBOT_ID | tag22 | ID van de robot (ArUco marker ID 2) |
| MQTT_HOST | jetson-dang.local | Adres van de MQTT broker |
| MQTT_PORT | 1883 | Poort van de broker |
| PUBLISH_INTERVAL | 5.0 sec | Hoe vaak data verstuurd wordt |

## MQTT Topics
| Topic | Inhoud | Formaat |
|-------|--------|---------|
| city/robots/tag22/battery | Batterij percentage | Integer (0-100) |
| city/robots/tag22/docking | Volledige status | JSON |

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

## Installatie
```bash
pip install paho-mqtt --break-system-packages
```

## Broker hostname fix
Als jetson-dang.local niet bereikbaar is, voer dit eenmalig uit op de Raspberry Pi:
```bash
echo "10.2.172.131 jetson-dang.local" | sudo tee -a /etc/hosts
```

## Manueel starten
```bash
source ~/ros2_ws/install/setup.bash
ROBOT_ID=tag22 ros2 run docking_robot mqtt_bridge
```

## Auto-start bij opstart (systemd)
De MQTT bridge start automatisch bij elke opstart van de Raspberry Pi en herstart automatisch bij crash.

### Service installeren
```bash
sudo nano /etc/systemd/system/mqtt_bridge.service
```

Plak dit erin:
```ini
[Unit]
Description=MQTT Bridge Docking Robot
After=network.target

[Service]
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ROBOT_ID=tag22 ros2 run docking_robot mqtt_bridge"
Restart=always
RestartSec=5
User=ubuntu

[Install]
WantedBy=multi-user.target
```

### Service activeren
```bash
sudo systemctl enable mqtt_bridge
sudo systemctl start mqtt_bridge
sudo systemctl status mqtt_bridge
```

### Service controleren
```bash
# Status bekijken
sudo systemctl status mqtt_bridge

# Logs bekijken
journalctl -u mqtt_bridge -f

# Herstarten
sudo systemctl restart mqtt_bridge

# Stoppen
sudo systemctl stop mqtt_bridge
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /battery_state | sensor_msgs/BatteryState | Subscriber |
