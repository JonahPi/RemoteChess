# FSD Remote Chess

## Summary

Remote Chess allows two board players to compete online via a MQTT Broker. Lifting and placing of figures is detected by hall-sensors underneath the squares. The moves are published in an MQTT-topic called /remotechess. Boards subscribed to the topic get the moves from the topic and indicate them with NeoPixel LEDs which are integrated in the squares. Lifting a figure is indicated in red, placing a figure is indicated in green.

## Board

The board has 8 times 8 fields setup in columns A to H and rows 1 to 8. Each field has a hall-sensor and a neopixel RGB LED.

The 64 Neopixel are organized in 8 rows so Fields A1 to H1 have the Neopixel numbers 1 to 8, the fields A2 to H2 have the Neopixel numbers 9 to 18 and so on till the fields A8 to H8 with Neopixel numbers 56 to 64.

The status of 64 hall sensors is detected by 8 PCBs, each of which has a MCP23017 GPIO Board Expander and 8 hall sensors to cover 1 row on the board. The 8 boards are all connected to an I2C bus. The board for the first row has the I2C address 0x20, the board for the second row has the address 0x21 and so on till the last row with board-address 0x27. The hall - sensors are connected to the  Inputs GPA0 to GPA7 of the Board expander, GPA0 corresponds to column A, GPA7 corresponds to column 7.

## Programming

### Detecting and publishing movement

When initializing the program, the following 4 states should be safed:
LastMoveLift: LML="", LastMovePlace: LMP="", LastMoveKilled: LMK="", FigureInAir = *false*,

If a hall-sensor changes the status from on (figure on field) to off (figure removed from field) and the status FigureInAir is equal to false, the action shall be to translate the sensor number to the correct field coordinate, upate LML  with the corresponding coordinate (e.g. LML = A4), set FigureInAir to true and publish the topic: *coordinate*-L (e.g. "A4-L" for a figure on field A4 which has been lifted in the air)

When the hall-sensor changes from *off* to *on*, the corresponding coordinate shall be stored in LMP (e.g. "B5"), the boolean FigureInAir shall be set to *false* and the message "*coordinate*-P" shall be published

If a hall status changes from on to off and FigureInAir is equal to *true* then the  coordinate shall be stored in status LMK and the topic "*coordinate*-X" shall be published.

### Receiving and Indicating movement

New messages on topic /remotechess shall trigger the following actions:

| Message format | Action                                                       |
| -------------- | ------------------------------------------------------------ |
| *coordinate*-L | 1. switch the Neopixel correesponding to *coordinate* to red<br />2. set neopixels corresponding to coordinate stored in LML, LMP and LMX to *off* (skip the neopixel if the variable is empty)<br />3. set values in LMP and LMX to ""<br />4. set value of LML to *coordinate*<br />5. stop the timer "fading" |
| *coordinate*-P | 1. switch the neopixel correspondig to coordinate to green<br />2. reset/start the timer "fading" and set it to 20s<br />3. set value of LMP to *coordinate*<br />4. if LMX is not empty, set it to empty and stop the blinking |
| *coordinate*-X | 1. switch the neopixel corresponding to *coordinate* to red blinking (200ms frequency)<br />2. set value of LMX to *coordinate* |

When the timer fading expires, the neopixels corresponding to the coordinates stored in LMP, LML and LMX shall be switched to *off* and all LEDs shall be switched to off.

### Pin-assignment

The program shall be written in Micropython and suitable for the development board Xiao ESP32-C6. Use the following pins:

| Function | Pin Number |
| -------- | ---------- |
| SCL      | D5         |
| SDA      | D4         |
| Neopixel | D7         |

### Secrets file

The access data (passwords, usernames, keys and IP-addresses) for the the Wifi and to the MQTT broker shall be stored in a dedicated secrets - file.

Time values, Blink-frequency and LED-colors shall be settable at the top of the pro

### Demo mode

Add a flag DemoMode which can be set at the beginning of the program. In demo-mode the hall-sensors are not connected and no expander boards. In this mode the program shall just check for the updates on the MQTT topic and control the neopixel stripe with 64 LEDs. The received updates and the color and number of the activated neopixel shall be shown in the serial monitor.