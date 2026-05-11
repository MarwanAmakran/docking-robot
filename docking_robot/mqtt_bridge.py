import json
import os
import time
import paho.mqtt.client as mqtt

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

ROBOT_ID  = os.environ.get("ROBOT_ID", "tag2")
MQTT_HOST = "jetson-dang.local"
MQTT_PORT = 1883

TOPIC_BATTERY = f"city/robots/{ROBOT_ID}/battery"
TOPIC_DOCKING = f"city/robots/{ROBOT_ID}/docking"

VOLTAGE_MIN      = 9.5
VOLTAGE_MAX      = 12.6
PUBLISH_INTERVAL = 5.0

_state = {
    "voltage"        : 0.0,
    "battery_pct"    : -1,
    "docking_state"  : "IDLE",
    "charging_status": "STABIEL",
    "prev_voltage"   : 0.0,
}
_mqtt_client = None

def voltage_to_pct(voltage):
    if voltage <= 0:
        return -1
    pct = (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0
    return max(0, min(100, int(round(pct))))

def detect_charging(voltage, prev):
    diff = voltage - prev
    if diff > 0.05:
        return "OPLADEN"
    elif diff < -0.05:
        return "ONTLADEN"
    return "STABIEL"

def mqtt_connect():
    global _mqtt_client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def on_connect(c, ud, flags, rc, props=None):
        if rc == 0:
            print(f"[MQTT] Verbonden met {MQTT_HOST}:{MQTT_PORT}")
        else:
            print(f"[MQTT] Verbinding mislukt (code {rc})")

    client.on_connect = on_connect
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    try:
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    except Exception as e:
        print(f"[MQTT] Verbinding mislukt: {e}")
    client.loop_start()
    _mqtt_client = client

def publish_state():
    if _mqtt_client is None:
        return
    pct = _state["battery_pct"]
    if pct >= 0:
        _mqtt_client.publish(TOPIC_BATTERY, str(pct), qos=0)
    payload = json.dumps({
        "state"   : _state["docking_state"],
        "charging": _state["charging_status"],
        "voltage" : round(_state["voltage"], 2),
        "pct"     : pct,
    })
    _mqtt_client.publish(TOPIC_DOCKING, payload, qos=0)
    print(f"[Bridge] bat={pct}% ({_state['voltage']:.2f}V) | dock={_state['docking_state']} | {_state['charging_status']}")

if ROS2_AVAILABLE:
    class DockingBridgeNode(Node):
        def __init__(self):
            super().__init__("docking_mqtt_bridge")
            self.sub_battery = self.create_subscription(
                BatteryState, "/battery_state", self._on_battery, 10)
            self.timer = self.create_timer(PUBLISH_INTERVAL, self._publish_timer)
            self.get_logger().info(f"[Bridge] Gestart voor robot {ROBOT_ID}")

        def _on_battery(self, msg):
            voltage = float(msg.voltage)
            prev    = _state["voltage"]
            _state["prev_voltage"]    = prev
            _state["voltage"]         = voltage
            _state["battery_pct"]     = voltage_to_pct(voltage)
            _state["charging_status"] = detect_charging(voltage, prev)

        def _publish_timer(self):
            publish_state()

def main():
    print(f"[Bridge] Starten voor {ROBOT_ID} | broker: {MQTT_HOST}:{MQTT_PORT}")
    mqtt_connect()
    time.sleep(1)

    if ROS2_AVAILABLE:
        rclpy.init()
        node = DockingBridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if _mqtt_client:
        _mqtt_client.loop_stop()
        _mqtt_client.disconnect()

if __name__ == "__main__":
    main()