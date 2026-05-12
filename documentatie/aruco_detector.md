# ArUco Detector

## Wat doet het?
Detecteert ArUco markers in de camerabeelden en toont de positie, fout en grootte van de marker in de terminal. Wordt gebruikt om te testen en kalibreren.

## ArUco instellingen
| Parameter | Waarde |
|-----------|--------|
| Dictionary | DICT_4X4_50 |
| Marker ID | 2 |
| Echte marker grootte | 100mm (10cm) |

## Output voorbeeld
```
Marker ID: 2 | Midden: 320 | Error: 0px | Grootte: 83px
```

## Uitleg output
| Waarde | Beschrijving |
|--------|-------------|
| Midden | X positie van marker in beeld (pixels) |
| Error | Verschil met beeldmidden (negatief = links, positief = rechts) |
| Grootte | Gemiddelde grootte in pixels (groter = dichter bij) |

## Afstand kalibratie
Op 28-30cm afstand = 83px grootte.
Op 6cm afstand = 430px grootte (docking afstand).

## Starten
```bash
source ~/ros2_ws/install/setup.bash
ros2 run docking_robot camera_publisher &
ros2 run docking_robot aruco_detector
```

## Topics
| Topic | Type | Richting |
|-------|------|---------|
| /camera/image_raw | sensor_msgs/Image | Subscriber |
