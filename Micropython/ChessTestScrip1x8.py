from machine import Pin, I2C
import neopixel
import time

# Configuration constants
frequency = 500  # Configurable frequency in ms for RGB test sequence
BRIGHTNESS = 0.4  # 40% brightness for hall-sensor test

# I2C-Setup für MCP23017 (SCL=D5, SDA=D4)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# Neopixel-Setup (8 LEDs auf Pin D7)
NUM_PIXELS = 8
np = neopixel.NeoPixel(Pin(7), NUM_PIXELS)

# MCP23017 Register addresses
IODIRA = 0x00  # I/O Direction Register für Port A
GPIOA  = 0x12  # Input Register für Port A

# Valid I2C addresses for the 8 PCBs
VALID_ADDRESSES = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]

def clear_all_leds():
    """Turn off all Neopixels"""
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
    """Scan for MCP23017 at addresses 0x20-0x27 and return found address"""
    print("=" * 40)
    print("STEP 1: I2C Address Detection")
    print("=" * 40)

    found_address = None
    devices = i2c.scan()

    for addr in VALID_ADDRESSES:
        if addr in devices:
            found_address = addr
            print(f"Found MCP23017 at address: 0x{addr:02X}")
            break

    if found_address is None:
        print("ERROR: No MCP23017 found at addresses 0x20-0x27!")
        print(f"Devices found on I2C bus: {['0x{:02X}'.format(d) for d in devices]}")

    print()
    return found_address

# =============================================================================
# STEP 2: Neopixel RGB Test Sequence
# =============================================================================
def step2_neopixel_rgb_test():
    """Test each Neopixel with RGB sequence: red -> green -> blue -> off"""
    print("=" * 40)
    print("STEP 2: Neopixel RGB Test Sequence")
    print(f"Frequency: {frequency}ms per color")
    print("=" * 40)

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
            print(f"  LED {led_num + 1} -> {color_name}")
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
def step3_hall_sensor_loop(mcp_address):
    """Continuously detect hall-sensors, show green LED at 40% brightness"""
    print("=" * 40)
    print("STEP 3: Hall-Sensor Detection Loop")
    print(f"Brightness: {int(BRIGHTNESS * 100)}%")
    print("Press Ctrl+C to exit")
    print("=" * 40)

    # Configure MCP23017: Set all GPA0-GPA7 as inputs
    i2c.writeto_mem(mcp_address, IODIRA, bytes([0xFF]))

    # Green color at 40% brightness
    green_40pct = scale_brightness((0, 255, 0), BRIGHTNESS)

    last_sensor_states = None

    while True:
        # Read current state of GPA0-GPA7
        sensor_states = int.from_bytes(i2c.readfrom_mem(mcp_address, GPIOA, 1), 'little')

        # Find sensors with detected magnets (LOW = magnet detected due to pull-up)
        detected_sensors = []
        for i in range(8):
            if (sensor_states & (1 << i)) == 0:  # LOW = magnet detected
                detected_sensors.append(i + 1)  # Sensor numbers 1-8

        # Update Neopixels: green at 40% brightness for detected magnets
        for i in range(NUM_PIXELS):
            if (sensor_states & (1 << i)) == 0:  # Magnet detected
                np[i] = green_40pct
            else:
                np[i] = (0, 0, 0)  # LED off
        np.write()

        # Print detected sensor numbers if state changed
        if sensor_states != last_sensor_states:
            if detected_sensors:
                print(f"Magnets detected at sensors: {detected_sensors}")
            else:
                print("No magnets detected")
            last_sensor_states = sensor_states

        time.sleep_ms(100)

# =============================================================================
# MAIN TEST PROGRAM
# =============================================================================
def main():
    print()
    print("*" * 40)
    print("  MCP23017 + Neopixel Module Test")
    print("  for Remote Chess PCB (1x8)")
    print("*" * 40)
    print()

    # Clear all LEDs at start
    clear_all_leds()

    # STEP 1: Scan for I2C address
    mcp_address = step1_scan_i2c()

    if mcp_address is None:
        print("Test aborted: No MCP23017 found.")
        return

    # STEP 2: Neopixel RGB test sequence
    step2_neopixel_rgb_test()

    # STEP 3: Hall-sensor continuous detection loop
    step3_hall_sensor_loop(mcp_address)

# Run the test program
if __name__ == "__main__":
    main()
