"""
FSD Remote Chess - Secrets Configuration File
Copy this file as 'secrets.py' and fill in your credentials
"""

# WiFi Configuration
WIFI_SSID = "your_wifi_ssid"
WIFI_PASSWORD = "your_wifi_password"

# MQTT Broker Configuration
MQTT_BROKER = "broker.example.com"  # IP address or hostname
MQTT_PORT = 1883                     # Default MQTT port
MQTT_USER = "your_mqtt_username"     # Leave empty string "" if no auth
MQTT_PASSWORD = "your_mqtt_password" # Leave empty string "" if no auth
MQTT_CLIENT_ID = "remote_chess_board_1"  # Unique ID for this board
MQTT_TOPIC = "ToniTwn/feeds/chess"   # MQTT topic for chess moves
