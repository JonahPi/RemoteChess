from machine import Pin, I2C
import neopixel
import time

# Configuration constants
frequency = 50  # Configurable frequency in ms for RGB test sequence
BRIGHTNESS = 0.3  # 40% brightness for hall-sensor test

# I2C-Setup für MCP23017 (SCL=D5=GPIO23, SDA=D4=GPIO22)
i2c = I2C(0, scl=Pin(23), sda=Pin(22), freq=100000)

# Neopixel-Setup - will be initialized after I2C scan
NEOPIXEL_PIN = Pin(18)
NUM_PIXELS = 0  # Will be set based on connected I2C devices
np = None  # Will be initialized after I2C scan

# MCP23017 Register addresses
IODIRA = 0x00  # I/O Direction Register für Port A
GPIOA  = 0x12  # Input Register für Port A

# Valid I2C addresses for the PCBs (0x20 to 0x27)
VALID_ADDRESSES = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]

def clear_all_leds():
    """Turn off all Neopixels"""
    if np is None:
        return
    for i in range(NUM_PIXELS):
        np[i] = (0, 0, 0)
    np.write()

def scale_brightness(color, brightness):
    """Scale RGB color by brightness factor (0.0 to 1.0)"""
    return tuple(int(c * brightness) for c in color)

# =============================================================================
# STEP 1: I2C Address Detection
# =============================================================================
def step1_scan_i2c():
    """Scan for MCP23017 at addresses 0x20-0x27, return list of found addresses"""
    global NUM_PIXELS, np

    print("=" * 40)
    print("STEP 1: I2C Address Detection")
    print("=" * 40)

    found_addresses = []
    devices = i2c.scan()

    # Check all permissible addresses from 0x20 to 0x27
    for addr in VALID_ADDRESSES:
        if addr in devices:
            found_addresses.append(addr)
            print(f"Found MCP23017 at address: 0x{addr:02X}")

    # Print total number of devices found
    device_count = len(found_addresses)
    print(f"\nTotal devices connected to I2C bus: {device_count}")

    # Calculate number of Neopixels (8 per device)
    NUM_PIXELS = device_count * 8
    print(f"Number of Neopixels available: {NUM_PIXELS}")

    if device_count == 0:
        print("ERROR: No MCP23017 found at addresses 0x20-0x27!")
        print(f"All devices found on I2C bus: {['0x{:02X}'.format(d) for d in devices]}")
    else:
        # Initialize Neopixel strip with calculated number of LEDs
        np = neopixel.NeoPixel(NEOPIXEL_PIN, NUM_PIXELS)

    print()
    return found_addresses

# =============================================================================
# STEP 2: Neopixel RGB Test Sequence
# =============================================================================
def step2_neopixel_rgb_test():
    """Test each Neopixel with RGB sequence: red -> green -> blue -> off"""
    print("=" * 40)
    print("STEP 2: Neopixel RGB Test Sequence")
    print(f"Frequency: {frequency}ms per color")
    print(f"Testing {NUM_PIXELS} Neopixels")
    print("=" * 40)

    if np is None or NUM_PIXELS == 0:
        print("ERROR: No Neopixels available (no I2C devices found)")
        print()
        return

    colors = [
        ((255, 0, 0), "red"),
        ((0, 255, 0), "green"),
        ((0, 0, 255), "blue")
    ]

    for led_num in range(NUM_PIXELS):
        print(f"Testing LED {led_num + 1}...")

        for color, color_name in colors:
            # Clear all LEDs first
            clear_all_leds()
            # Set only current LED to current color
            np[led_num] = color
            np.write()
            #print(f"  LED {led_num + 1} -> {color_name}")
            time.sleep_ms(frequency)

        # Turn off after RGB cycle
        clear_all_leds()
        print(f"  LED {led_num + 1} -> off")
        time.sleep_ms(frequency)

    print("Neopixel RGB test completed.")
    print()

# =============================================================================
# STEP 3: Hall-Sensor Continuous Detection Loop
# =============================================================================
def sensor_to_coordinate(row, col):
    """Convert row (0-7) and column (0-7) to chess coordinate (e.g., 'A1')"""
    column_letter = chr(ord('A') + col)
    row_number = row + 1
    return f"{column_letter}{row_number}"

def step3_hall_sensor_loop(mcp_addresses):
    """Continuously detect hall-sensors, print coordinates and light up LEDs"""
    print("=" * 40)
    print("STEP 3: Hall-Sensor Detection Loop")
    print(f"Monitoring {len(mcp_addresses)} MCP23017 device(s)")
    print(f"LED brightness: {int(BRIGHTNESS * 100)}%")
    print("Press Ctrl+C to exit")
    print("=" * 40)

    # Configure all MCP23017s: Set all GPA0-GPA7 as inputs
    for addr in mcp_addresses:
        i2c.writeto_mem(addr, IODIRA, bytes([0xFF]))

    # Green color at configured brightness
    green = scale_brightness((0, 255, 0), BRIGHTNESS)

    # Track previous detection state
    last_detection = None

    while True:
        detected_coordinates = []

        # Clear all LEDs first
        for i in range(NUM_PIXELS):
            np[i] = (0, 0, 0)

        # Read from all connected MCP23017s
        for addr in mcp_addresses:
            # Calculate row number from address (0x20 = row 0, 0x21 = row 1, etc.)
            row = addr - 0x20

            # Read current state of GPA0-GPA7
            sensor_states = int.from_bytes(i2c.readfrom_mem(addr, GPIOA, 1), 'little')

            # Find sensors with detected magnets (LOW = magnet detected due to pull-up)
            for col in range(8):
                if (sensor_states & (1 << col)) == 0:  # LOW = magnet detected
                    coord = sensor_to_coordinate(row, col)
                    detected_coordinates.append(coord)
                    # Light up corresponding LED in green
                    led_index = row * 8 + col
                    np[led_index] = green

        # Update LEDs
        np.write()

        # Print coordinates if detection changed
        current_state = tuple(detected_coordinates)
        if current_state != last_detection:
            if detected_coordinates:
                print(f"Magnets detected at: {', '.join(detected_coordinates)}")
            else:
                print("No magnets detected")
            last_detection = current_state

        time.sleep_ms(100)

# =============================================================================
# MAIN TEST PROGRAM
# =============================================================================
def main():
    print()
    print("*" * 40)
    print("  MCP23017 + Neopixel Module Test")
    print("  for Remote Chess PCB")
    print("*" * 40)
    print()

    # STEP 1: Scan for I2C addresses
    mcp_addresses = step1_scan_i2c()

    if not mcp_addresses:
        print("Test aborted: No MCP23017 found.")
        return

    # Clear all LEDs at start (now that np is initialized)
    clear_all_leds()

    # STEP 2: Neopixel RGB test sequence
    step2_neopixel_rgb_test()

    # STEP 3: Hall-sensor continuous detection loop (all found addresses)
    step3_hall_sensor_loop(mcp_addresses)

# Run the test program
if __name__ == "__main__":
    main()
