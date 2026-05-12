# Camera Publisher

## Wat doet het?
Leest de camera uit en publiceert de beelden op het /camera/image_raw topic zodat andere nodes ze kunnen gebruiken voor ArUco detectie.

## Instellingen
| Parameter | Waarde |
|-----------|--------|
| FPS | 25 frames per seconde |
| Camera index | 0 (eerste camera) |
| Formaat | BGR8 |

## Starten
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot camera_publisher
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /camera/image_raw | sensor_msgs/Image | Publisher |
