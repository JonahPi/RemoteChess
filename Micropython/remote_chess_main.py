"""
FSD Remote Chess - Main Program
For Xiao ESP32-C6 with MicroPython
"""

import network
import time
import json
import socket
from machine import Pin, I2C, Timer
from neopixel import NeoPixel
from umqtt.simple import MQTTClient
from secrets import *

# ============= CONFIGURATION =============
# Demo Mode - Set to True to run without hall sensors
DEMO_MODE = False  # Set to True for testing without hardware

# LED Colors (R, G, B)
COLOR_LIFT = (100, 0, 0)      # Red for lifted pieces
COLOR_PLACE = (0, 100, 0)     # Green for placed pieces
COLOR_KILLED = (100, 0, 0)    # Red for killed pieces
COLOR_OFF = (0, 0, 0)         # Off

# Timing
FADING_TIMEOUT_MS = 15000     # 20 seconds
BLINK_FREQUENCY_MS = 400      # 200ms for blinking
STARTUP_ROW_DURATION_MS = 800  # 800ms per row during startup

# Startup brightness (40% = 0.4)
STARTUP_BRIGHTNESS = 0.4

# Pin Configuration (Xiao ESP32-C6)
PIN_SDA = 22  # D4 = GPIO22
PIN_SCL = 23  # D5 = GPIO23
PIN_NEOPIXEL = 18  # D10 = GPIO18

# Board Configuration
NUM_NEOPIXELS = 64
MCP23017_ADDRESSES = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]

# ============= GLOBAL VARIABLES =============
LML = ""  # Last Move Lift
LMP = ""  # Last Move Place
LMK = ""  # Last Move Killed (coordinate with blinking LED)

# State machine variables per FSD
FiguresInAir = 0  # Counter: 0, 1, or 2
first_lift_coord = ""  # Coordinate of first lifted piece (c1)
second_lift_coord = ""  # Coordinate of second lifted piece (c2)

# Hardware objects
i2c = None
np = None
mqtt_client = None
fading_timer = None
blink_timer = None
blink_state = False

# Track previous hall sensor states (only used in non-demo mode)
previous_hall_states = [[False] * 8 for _ in range(8)]

# MQTT keep-alive tracking
last_mqtt_activity = 0  # Reset on any MQTT publish

# ============= MCP23017 Functions =============
def mcp23017_init(i2c, address):
    """Initialize MCP23017 - set all pins as inputs with pull-ups"""
    # IODIRA register (0x00) - set all as inputs (0xFF)
    i2c.writeto_mem(address, 0x00, b'\xFF')
    # GPPUA register (0x0C) - enable pull-ups
    i2c.writeto_mem(address, 0x0C, b'\xFF')

def mcp23017_read_gpio(i2c, address):
    """Read GPIOA register (0x12) from MCP23017"""
    data = i2c.readfrom_mem(address, 0x12, 1)
    return data[0]

# ============= Coordinate Conversion =============
def sensor_to_coordinate(row_idx, col):
    """Convert row_idx (0-7) and column (0-7) to chess coordinate.

    Note: I2C addresses are mapped as follows:
    - row_idx 0 (I2C 0x20) = Chess Row 8
    - row_idx 7 (I2C 0x27) = Chess Row 1
    """
    column_letter = chr(ord('A') + col)
    row_number = 8 - row_idx  # row_idx 0 -> row 8, row_idx 7 -> row 1
    return f"{column_letter}{row_number}"

def coordinate_to_neopixel(coord):
    """Convert chess coordinate (e.g., 'A1') to neopixel index (0-63).

    Note: Neopixels are counted from A8 (index 0) to H1 (index 63).
    - A8 = 0, H8 = 7
    - A1 = 56, H1 = 63
    """
    col = ord(coord[0]) - ord('A')
    chess_row = int(coord[1])
    row_idx = 8 - chess_row  # row 8 -> idx 0, row 1 -> idx 7
    return row_idx * 8 + col

# ============= NeoPixel Functions =============
def set_neopixel(coord, color):
    """Set neopixel at coordinate to specified color"""
    if coord:
        idx = coordinate_to_neopixel(coord)
        np[idx] = color
        np.write()
        if DEMO_MODE:
            print(f"[DEMO] NeoPixel #{idx} ({coord}) -> RGB{color}")

def clear_neopixels(coords):
    """Clear neopixels at given coordinates"""
    for coord in coords:
        if coord:
            set_neopixel(coord, COLOR_OFF)

def blink_callback(timer):
    """Timer callback for blinking effect"""
    global blink_state, LMK
    if LMK:
        blink_state = not blink_state
        color = COLOR_KILLED if blink_state else COLOR_OFF
        set_neopixel(LMK, color)

def fading_callback(timer):
    """Timer callback for fading timeout"""
    global LML, LMP, LMK
    if DEMO_MODE:
        print(f"[DEMO] Fading timeout - clearing all LEDs")
    # Clear specific coordinates
    clear_neopixels([LML, LMP, LMK])
    # Clear all LEDs
    for i in range(NUM_NEOPIXELS):
        np[i] = COLOR_OFF
    np.write()
    # Reset state variables
    LML = ""
    LMP = ""
    LMK = ""
    if blink_timer:
        blink_timer.deinit()

# ============= MQTT Functions =============
def mqtt_callback(topic, msg):
    """Handle incoming MQTT messages"""
    global LML, LMP, LMK, fading_timer, blink_timer, previous_hall_states

    message = msg.decode('utf-8')
    print(f"Received: {message}")

    # Handle restart message
    if message == "restart":
        global FiguresInAir, first_lift_coord, second_lift_coord
        print("Restart triggered - running startup sequence")
        # Clear all state
        LML = ""
        LMP = ""
        LMK = ""
        FiguresInAir = 0
        first_lift_coord = ""
        second_lift_coord = ""
        if fading_timer:
            fading_timer.deinit()
        if blink_timer:
            blink_timer.deinit()
        # Clear all LEDs
        for i in range(NUM_NEOPIXELS):
            np[i] = COLOR_OFF
        np.write()
        # Run startup sequence and re-read hall sensor states
        if not DEMO_MODE:
            startup_led_test()
            # Re-initialize hall sensor states after restart
            for row_idx, address in enumerate(MCP23017_ADDRESSES):
                try:
                    gpio_state = mcp23017_read_gpio(i2c, address)
                    for col_idx in range(8):
                        previous_hall_states[row_idx][col_idx] = (gpio_state & (1 << col_idx)) == 0
                except Exception as e:
                    print(f"Error reading state after restart: {e}")
        return

    if len(message) < 4:
        return

    coord = message[:-2]  # Extract coordinate (e.g., 'A4' from 'A4-L')
    action = message[-1]  # Extract action ('L', 'P', 'R', or 'X')
    
    if action == 'L':
        # Lift action
        if DEMO_MODE:
            print(f"[DEMO] Lift detected at {coord}")
        set_neopixel(coord, COLOR_LIFT)
        clear_neopixels([LML, LMP, LMK])
        LMP = ""
        LMK = ""
        LML = coord
        if fading_timer:
            fading_timer.deinit()
    
    elif action == 'P':
        # Place action
        if DEMO_MODE:
            print(f"[DEMO] Place detected at {coord}")
        set_neopixel(coord, COLOR_PLACE)
        LMP = coord
        # If LMK is not empty, clear it and stop blinking
        if LMK:
            if DEMO_MODE:
                print(f"[DEMO] Clearing killed piece at {LMK} and stopping blink")
            LMK = ""
            if blink_timer:
                blink_timer.deinit()
        if fading_timer:
            fading_timer.deinit()
        fading_timer = Timer(0)
        fading_timer.init(mode=Timer.ONE_SHOT, period=FADING_TIMEOUT_MS, callback=fading_callback)

    elif action == 'R':
        # Return action - figure returned to original position
        if DEMO_MODE:
            print(f"[DEMO] Figure returned to {coord}")
        set_neopixel(coord, COLOR_OFF)
        LML = ""

    elif action == 'X':
        # Killed action
        if DEMO_MODE:
            print(f"[DEMO] Piece killed at {coord}")
        LMK = coord
        if blink_timer:
            blink_timer.deinit()
        blink_timer = Timer(1)
        blink_timer.init(mode=Timer.PERIODIC, period=BLINK_FREQUENCY_MS, callback=blink_callback)
        # Start fading timer so blinking LED turns off after timeout
        if fading_timer:
            fading_timer.deinit()
        fading_timer = Timer(0)
        fading_timer.init(mode=Timer.ONE_SHOT, period=FADING_TIMEOUT_MS, callback=fading_callback)

def publish_move(message):
    """Publish move to MQTT topic"""
    global last_mqtt_activity
    try:
        mqtt_client.publish(MQTT_TOPIC, message)
        last_mqtt_activity = time.time()  # Reset ping timer
        print(f"Published: {message}")
    except Exception as e:
        print(f"MQTT publish error: {e}")

# ============= Hall Sensor Scanning =============
def scan_hall_sensors():
    """Scan all hall sensors and detect changes.

    State machine per FSD:
    - State 1: FIA=0, waiting for first lift
    - State 2: FIA=1, one piece lifted (c1)
    - State 3: FIA=0, piece placed (move complete)
    - State 4: FIA=2, two pieces lifted (capture scenario)

    Transitions:
    - State 1 → 2: lift → FIA=1, send "c1-L"
    - State 2 → 3: place at new coord → FIA=0, send "c2-P"
    - State 2 → 4: lift second piece → FIA=2, send "c2-L"
    - State 4 → restart: place → FIA=0, send "cX-X" (capture)
    - State 2 → restart: place back at c1 → FIA=0, send "c1-P"

    When LEDs indicate a move from the other player (LML/LMP set),
    synchronizing figures to follow that move does NOT trigger MQTT messages.
    """
    global FiguresInAir, first_lift_coord, second_lift_coord, previous_hall_states, LML, LMP

    for row_idx, address in enumerate(MCP23017_ADDRESSES):
        try:
            gpio_state = mcp23017_read_gpio(i2c, address)

            for col_idx in range(8):
                # Check if bit is LOW (0 = magnet detected = figure present, with pull-ups)
                current_state = (gpio_state & (1 << col_idx)) == 0
                previous_state = previous_hall_states[row_idx][col_idx]

                # Detect state change
                if current_state != previous_state:
                    coord = sensor_to_coordinate(row_idx, col_idx)

                    # Figure removed (was present, now absent)
                    if previous_state and not current_state:
                        # Check if this is synchronizing to an indicated move (lift from LML)
                        if coord == LML:
                            # Player is following indicated move - don't publish
                            # Turn off the red LED and clear LML
                            # Note: Clearing LML here is necessary for sync scenarios since
                            # no MQTT message is published (FSD note applies to self-received messages)
                            print(f"Sync lift at {coord} (following indicated move)")
                            set_neopixel(coord, COLOR_OFF)
                            LML = ""
                        elif coord == LMP:
                            # Player is removing killed piece from destination - no action
                            # Green LED stays on until new piece is placed
                            print(f"Sync kill removal at {coord} (removing captured piece)")
                        elif FiguresInAir == 0:
                            # State 1 → State 2: First lift
                            FiguresInAir = 1
                            first_lift_coord = coord
                            publish_move(f"{coord}-L")
                        elif FiguresInAir == 1:
                            # State 2 → State 4: Second lift (capture scenario)
                            FiguresInAir = 2
                            second_lift_coord = coord
                            publish_move(f"{coord}-L")  # FSD: send L, not X

                    # Figure placed (was absent, now present)
                    elif not previous_state and current_state:
                        # Check if this is synchronizing to an indicated move (place at LMP)
                        if coord == LMP:
                            # Player is following indicated move - don't publish
                            # Turn off the green LED and clear LMP
                            print(f"Sync place at {coord} (following indicated move)")
                            set_neopixel(coord, COLOR_OFF)
                            LMP = ""
                        elif FiguresInAir == 1:
                            # State 2: One piece in air
                            if coord == first_lift_coord:
                                # State 2 → restart: Return to original position
                                FiguresInAir = 0
                                first_lift_coord = ""
                                publish_move(f"{coord}-R")  # FSD: send R for return
                            else:
                                # State 2 → State 3 → restart: Normal move
                                FiguresInAir = 0
                                first_lift_coord = ""
                                publish_move(f"{coord}-P")
                        elif FiguresInAir == 2:
                            # State 4 → restart: Capture placement
                            # When placing while 2 pieces are in air, this is a capture
                            FiguresInAir = 0
                            first_lift_coord = ""
                            second_lift_coord = ""
                            publish_move(f"{coord}-X")  # FSD: send X for capture

                    previous_hall_states[row_idx][col_idx] = current_state

        except Exception as e:
            print(f"Error reading MCP23017 at 0x{address:02X}: {e}")

# ============= Startup Routine =============
def startup_led_test():
    """Display hall sensor status row by row during startup"""
    print("Running startup LED test...")

    # Calculate dimmed colors (40% brightness)
    color_red = tuple(int(c * STARTUP_BRIGHTNESS) for c in COLOR_LIFT)
    color_green = tuple(int(c * STARTUP_BRIGHTNESS) for c in COLOR_PLACE)

    for row_idx, address in enumerate(MCP23017_ADDRESSES):
        try:
            # Read hall sensor state for this row
            gpio_state = mcp23017_read_gpio(i2c, address)

            # Light up LEDs for this row
            for col_idx in range(8):
                # Check if magnet detected (LOW = magnet present)
                magnet_present = (gpio_state & (1 << col_idx)) == 0
                led_idx = row_idx * 8 + col_idx

                if magnet_present:
                    np[led_idx] = color_red    # Red for magnet/figure present
                else:
                    np[led_idx] = color_green  # Green for no magnet/empty

            np.write()
            print(f"Row {row_idx + 1} (0x{address:02X}): displayed")

            # Wait 1 second
            time.sleep(STARTUP_ROW_DURATION_MS / 1000)

            # Turn off this row before next
            for col_idx in range(8):
                led_idx = row_idx * 8 + col_idx
                np[led_idx] = COLOR_OFF
            np.write()

        except Exception as e:
            print(f"Error in startup test row {row_idx + 1}: {e}")

    print("Startup LED test complete")

# ============= WiFi Manager =============
WIFI_CONFIG_FILE = "wifi_config.json"
AP_SSID = "RemoteChess-Setup"

def url_decode(s):
    """Simple URL decoder for form data"""
    s = s.replace('+', ' ')
    parts = s.split('%')
    result = parts[0]
    for part in parts[1:]:
        if len(part) >= 2:
            try:
                result += chr(int(part[:2], 16)) + part[2:]
            except ValueError:
                result += '%' + part
        else:
            result += '%' + part
    return result

def load_wifi_config():
    """Load WiFi credentials from permanent storage"""
    try:
        with open(WIFI_CONFIG_FILE, 'r') as f:
            config = json.load(f)
            return config.get('ssid', ''), config.get('password', '')
    except:
        return '', ''

def save_wifi_config(ssid, password):
    """Save WiFi credentials to permanent storage"""
    config = {'ssid': ssid, 'password': password}
    with open(WIFI_CONFIG_FILE, 'w') as f:
        json.dump(config, f)
    print(f"WiFi config saved for network: {ssid}")

def scan_wifi_networks():
    """Scan for available WiFi networks"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    networks = wlan.scan()
    wlan.active(False)
    seen = set()
    unique = []
    for net in networks:
        ssid = net[0].decode('utf-8') if isinstance(net[0], bytes) else net[0]
        if ssid and ssid not in seen:
            seen.add(ssid)
            unique.append((ssid, net[3]))
    unique.sort(key=lambda x: x[1], reverse=True)
    return unique

def generate_wifi_html(networks):
    """Generate HTML page for WiFi network selection"""
    options = ''
    for ssid, rssi in networks:
        options += '<option value="' + ssid + '">' + ssid + ' (' + str(rssi) + ' dBm)</option>'
    return ('<!DOCTYPE html><html><head>'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        '<title>Remote Chess WiFi Setup</title>'
        '<style>'
        'body{font-family:Arial;margin:20px;background:#1a1a2e;color:#eee;}'
        'h1{color:#e94560;}h2{color:#aaa;}'
        'select,input,button{width:100%;padding:12px;margin:8px 0;'
        'box-sizing:border-box;font-size:16px;border-radius:4px;border:1px solid #555;}'
        'select,input{background:#16213e;color:#eee;}'
        'button{background:#e94560;color:white;border:none;cursor:pointer;}'
        '.c{max-width:400px;margin:0 auto;}'
        '</style></head><body><div class="c">'
        '<h1>Remote Chess</h1><h2>WiFi Setup</h2>'
        '<form action="/connect" method="POST">'
        '<label>Select Network:</label>'
        '<select name="ssid">' + options + '</select>'
        '<label>Password:</label>'
        '<input type="password" name="password" placeholder="Enter WiFi password">'
        '<button type="submit">Connect</button>'
        '</form></div></body></html>')

def start_wifi_manager():
    """Start WiFi Manager AP with web portal for network selection"""
    print("Scanning WiFi networks...")
    networks = scan_wifi_networks()
    print(f"Found {len(networks)} networks")

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID)

    print(f"WiFi Manager AP started: {AP_SSID}")
    print("Connect to this network and open http://192.168.4.1")

    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)

    html_page = generate_wifi_html(networks)

    try:
        while True:
            cl, cl_addr = s.accept()
            try:
                request = cl.recv(1024).decode('utf-8')

                if 'POST /connect' in request:
                    body = request.split('\r\n\r\n')[-1]
                    params = {}
                    for param in body.split('&'):
                        if '=' in param:
                            key, value = param.split('=', 1)
                            params[key] = url_decode(value)

                    ssid = params.get('ssid', '')
                    password = params.get('password', '')

                    if ssid:
                        response = ('<!DOCTYPE html><html><head>'
                            '<meta name="viewport" content="width=device-width, initial-scale=1">'
                            '<style>body{font-family:Arial;margin:20px;background:#1a1a2e;'
                            'color:#eee;text-align:center;}h1{color:#e94560;}</style>'
                            '</head><body>'
                            '<h1>Connecting to ' + ssid + '...</h1>'
                            '<p>The board will connect to the selected network.</p>'
                            '<p>You can close this page.</p>'
                            '</body></html>')
                        cl.send('HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n')
                        cl.send(response)
                        cl.close()

                        save_wifi_config(ssid, password)
                        ap.active(False)
                        s.close()
                        return ssid, password
                else:
                    cl.send('HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n')
                    cl.send(html_page)
            except Exception as e:
                print(f"WiFi Manager error: {e}")
            finally:
                try:
                    cl.close()
                except:
                    pass
    except Exception as e:
        print(f"WiFi Manager server error: {e}")
        s.close()
        ap.active(False)
        raise

# ============= WiFi Connection =============
def connect_wifi():
    """Connect to WiFi with fallback to WiFi Manager hotspot"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # Build list of credentials to try: saved config first, then secrets file
    saved_ssid, saved_password = load_wifi_config()
    credentials = []
    if saved_ssid:
        credentials.append((saved_ssid, saved_password))
    if WIFI_SSID and WIFI_SSID != saved_ssid:
        credentials.append((WIFI_SSID, WIFI_PASSWORD))

    # Try each set of stored credentials
    for ssid, password in credentials:
        print(f'Trying WiFi: {ssid}...')
        try:
            wlan.connect(ssid, password)

            timeout = 10
            while not wlan.isconnected() and timeout > 0:
                time.sleep(1)
                timeout -= 1

            if wlan.isconnected():
                print('WiFi connected:', wlan.ifconfig())
                return

            print(f'Failed to connect to {ssid}')
            wlan.disconnect()
        except OSError as e:
            print(f'WiFi error for {ssid}: {e}')
            wlan.disconnect()

    # No stored credentials worked - start WiFi Manager hotspot
    print("No WiFi connection established - starting WiFi Manager hotspot...")
    wlan.active(False)

    while True:
        ssid, password = start_wifi_manager()

        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(ssid, password)

        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1

        if wlan.isconnected():
            print('WiFi connected:', wlan.ifconfig())
            return

        print(f'Failed to connect to {ssid} - restarting WiFi Manager...')
        wlan.active(False)

# ============= Main Program =============
def main():
    global i2c, np, mqtt_client
    
    print("=== FSD Remote Chess Starting ===")
    
    if DEMO_MODE:
        print("*** DEMO MODE ENABLED ***")
        print("Hall sensors disabled - MQTT receive only mode")
    
    # Initialize I2C (only if not in demo mode)
    if not DEMO_MODE:
        print("Initializing I2C...")
        i2c = I2C(0, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA), freq=100000)
        
        # Initialize MCP23017 chips
        print("Initializing MCP23017 chips...")
        for address in MCP23017_ADDRESSES:
            try:
                mcp23017_init(i2c, address)
                print(f"MCP23017 at 0x{address:02X} initialized")
            except Exception as e:
                print(f"Error initializing MCP23017 at 0x{address:02X}: {e}")
    
    # Initialize NeoPixels
    print("Initializing NeoPixels...")
    np = NeoPixel(Pin(PIN_NEOPIXEL), NUM_NEOPIXELS)
    for i in range(NUM_NEOPIXELS):
        np[i] = COLOR_OFF
    np.write()

    # Run startup LED test (only if not in demo mode)
    if not DEMO_MODE:
        startup_led_test()

    # Connect to WiFi
    connect_wifi()
    
    # Connect to MQTT
    print("Connecting to MQTT broker...")
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Client ID: {MQTT_CLIENT_ID}")

    try:
        mqtt_client = MQTTClient(
            MQTT_CLIENT_ID,
            MQTT_BROKER,
            port=MQTT_PORT,
            user=MQTT_USER if MQTT_USER else None,
            password=MQTT_PASSWORD if MQTT_PASSWORD else None
        )
        mqtt_client.set_callback(mqtt_callback)
        mqtt_client.connect()
        mqtt_client.subscribe(MQTT_TOPIC)
        print(f"MQTT connected and subscribed to {MQTT_TOPIC}")

        # Show connection indication: D4, E4 green and D5, E5 red for 1 second
        set_neopixel("D4", COLOR_PLACE)
        set_neopixel("E4", COLOR_PLACE)
        set_neopixel("D5", COLOR_LIFT)
        set_neopixel("E5", COLOR_LIFT)
        time.sleep(1)
        set_neopixel("D4", COLOR_OFF)
        set_neopixel("E4", COLOR_OFF)
        set_neopixel("D5", COLOR_OFF)
        set_neopixel("E5", COLOR_OFF)
        print("Ready to play!")
    except Exception as e:
        print(f"MQTT connection failed: {e}")
        print("Error codes: 1=Protocol, 2=ClientID, 3=Unavailable, 4=Auth, 5=Not authorized")
        print("Check your secrets.py file:")
        print("  - MQTT_BROKER address")
        print("  - MQTT_PORT (usually 1883)")
        print("  - MQTT_USER and MQTT_PASSWORD")
        print("  - Ensure broker allows this client ID")
        raise
    
    # Initialize hall sensor states (only if not in demo mode)
    if not DEMO_MODE:
        print("Reading initial hall sensor states...")
        for row_idx, address in enumerate(MCP23017_ADDRESSES):
            try:
                gpio_state = mcp23017_read_gpio(i2c, address)
                for col_idx in range(8):
                    # LOW (0) = magnet detected = figure present (with pull-ups)
                    previous_hall_states[row_idx][col_idx] = (gpio_state & (1 << col_idx)) == 0
            except Exception as e:
                print(f"Error reading initial state: {e}")
    
    print("=== Remote Chess Ready ===")
    if DEMO_MODE:
        print("Waiting for MQTT messages...")

    # Keep-alive ping interval (30 seconds, Adafruit IO uses 60s timeout)
    global last_mqtt_activity
    PING_INTERVAL = 30
    last_mqtt_activity = time.time()

    # Main loop
    while True:
        try:
            # Check for MQTT messages
            mqtt_client.check_msg()

            # Send keep-alive ping only when idle (no recent publish)
            if time.time() - last_mqtt_activity > PING_INTERVAL:
                mqtt_client.ping()
                last_mqtt_activity = time.time()

            # Scan hall sensors (only if not in demo mode)
            if not DEMO_MODE:
                scan_hall_sensors()

            # Small delay to avoid overwhelming the system
            time.sleep(0.05)

        except OSError as e:
            print(f"MQTT connection error: {e}")
            print("Reconnecting...")
            time.sleep(2)
            try:
                mqtt_client.connect()
                mqtt_client.subscribe(MQTT_TOPIC)
                last_mqtt_activity = time.time()
                print("Reconnected to MQTT")
            except Exception as reconnect_error:
                print(f"Reconnect failed: {reconnect_error}")
                time.sleep(5)

        except KeyboardInterrupt:
            print("\nShutting down...")
            if mqtt_client:
                mqtt_client.disconnect()
            clear_neopixels([LML, LMP, LMK])
            break

# Run the program
if __name__ == "__main__":
    main()