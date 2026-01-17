from machine import Pin, I2C
import neopixel
import time

# I2C-Setup für MCP23017 (SCL=D5, SDA=D4)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# Neopixel-Setup (8 LEDs auf Pin D7)
NUM_PIXELS = 8
np = neopixel.NeoPixel(Pin(7), NUM_PIXELS)

# MCP23017 I2C-Adresse (Standard: 0x20)
MCP_ADDRESS = 0x20

# Register-Adressen für MCP23017
IODIRA = 0x00  # I/O Direction Register für Port A
GPIOA  = 0x12  # Input Register für Port A

# Setze alle GPA0-GPA7 als Eingänge (Pull-up)
i2c.writeto_mem(MCP_ADDRESS, IODIRA, bytes([0xFF]))

last_sensor_states = 0

def print_sensor_states(states):
    print("sensorStates:", end=" ")
    for i in range(7, -1, -1):
        print("1" if states & (1 << i) else "0", end=" ")
    print()

def read_mcp_gpio():
    # Lese den aktuellen Zustand von GPA0-GPA7
    return int.from_bytes(i2c.readfrom_mem(MCP_ADDRESS, GPIOA, 1), 'little')

while True:
    sensor_states = read_mcp_gpio()

    # Drucke den Binärzustand
    print_sensor_states(sensor_states)

    # Prüfe auf Änderungen
    if sensor_states != last_sensor_states:
        for i in range(8):
            current_state = (sensor_states & (1 << i)) == 0  # LOW = Magnet erkannt (Pull-up)
            last_state = (last_sensor_states & (1 << i)) == 0
            if current_state != last_state:
                print(f"Hall-Sensor {i}: {'Magnet erkannt' if current_state else 'Kein Magnet'}")

    # Aktualisiere Neopixel
    for i in range(NUM_PIXELS):
        np[i] = (255, 0, 0) if (sensor_states & (1 << i)) == 0 else (0, 0, 0)
    np.write()

    last_sensor_states = sensor_states
    time.sleep_ms(100)
