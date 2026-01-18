# FSD Remote Chess

## Summary

Remote Chess allows two board players to compete online via a MQTT Broker. Lifting and placing of figures is detected by hall-sensors underneath the squares. The moves are published in an MQTT-topic called /remotechess. Boards subscribed to the topic get the moves from the topic and indicate them with NeoPixel LEDs which are integrated in the squares. Lifting a figure is indicated in red, placing a figure is indicated in green.

## Board

The board has 8 times 8 fields setup in columns A to H and rows 1 to 8. Each field has a hall-sensor and a neopixel RGB LED.

The 64 Neopixel are organized in 8 rows so Fields A1 to H1 have the Neopixel numbers 1 to 8, the fields A2 to H2 have the Neopixel numbers 9 to 18 and so on till the fields A8 to H8 with Neopixel numbers 56 to 64.

The status of 64 hall sensors is detected by 8 PCBs, each of which has a MCP23017 GPIO Board Expander and 8 hall sensors to cover 1 row on the board. The 8 boards are all connected to an I2C bus. The board for the first row has the I2C address 0x20, the board for the second row has the address 0x21 and so on till the last row with board-address 0x27. The hall - sensors are connected to the  Inputs GPA0 to GPA7 of the Board expander, GPA0 corresponds to column A, GPA7 corresponds to column 7.

#### Module test

The 8 PCBs shall be tested separately before the complete board assembly. To do this the I2C connections (SDA & SCL), the Neopixel data input (DIN) and the 5V input is directly connected to the XIAO ESP32-C6 (s.a. PIN assignement further below).

The test program has 3 steps:

1. Check if one of the permissible I2C addresses from 0x21 to 0x27 is set and print the address found in the serial terminal. 
2. Switch every single of the 8 Neopixel on and off in the 3 RGB colors, only one Neopixel at at a time shall be switched on. The frequency should be 500ms but configurable in a constant called frequency. So the test sequence looks like (Led 1 -> red -> green -> blue -> off, Led 2 -> red -> green -> blue -> off, ...and so on)
3. Switch in a loop which continuously detects the state of the hall-sensors, if a magnet is detected it should show the corresponding LED in green, if no magnet is detected the corresponding LED should be off. The brightness of the LEDs shall be reduced to 40%. Print the number of the hallsensors which detected a magnet in the serial window.

## ESP-Programming

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

### MQTT Broker

Use the following MQTT Broker Configuration

​    broker: 'broker.hivemq.com',
​    port: 8884,
​    useSSL: true,
​    topic: 'home/chess'

### Demo mode

Add a flag DemoMode which can be set at the beginning of the program. In demo-mode the hall-sensors are not connected and no expander boards. In this mode the program shall just check for the updates on the MQTT topic and control the neopixel stripe with 64 LEDs. The received updates and the color and number of the activated neopixel shall be shown in the serial monitor.

## **Progressive Web App (PWA**)

### Summary

The Web App allows the simulation of the real chess board. So one player can use the app and the other player uses the real board. Alternatively both players can use the app or both players can use a real board

### Architecture

The Web App shall be hosted as page in the Git-repository for this project ([Remote Chess](https://jonahpi.github.io/RemoteChess/)). The Web App shall simulate the Chess-board and shall be optimized for an iPhone 16 from the layout. By touching a figure ones it is activated for moving. Touching a field afterwards will place the figure there. In case the destination field was not empty, the destination figure is replaced with the figure which has been selected. When moving the figures no chess rules need to be considered. Touching the figure shall have the same effect as lifting a figure on the real chess box including the MQTT message. The LEDs shall be simulated by small dots on the simulated chess field. 

A button called "Zurück" allows to undo the latest action on App. It will move the figures which has been moved by the user back to the last position, the LEDs are switched off and in case another figure has been killed, the figure should be replaced to its original position. "Zurück" only needs to work for one move. the button shall be greyed out when it has been used and re-activated after the next move.

When the App receives a MQTT message of type *coordinate*-L which has not been triggered by touching a figure in the app itself, this means that the other player has lifted a figure. The App shall then wait for the message that a figure has been placed (*coordinate*-P) and move the corresponding icon from the lift-corrdinates to the place-coordinates.



### Icon Design

 All chess figures are depicted by simple icons. the Icons for one player shall be black, the ones for the other player white

1. **Black pieces** - Displayed in pure black color (#000)
2. **White pieces** - Displayed in white with a black outline/shadow so they're visible on light squares
3. **Consistent rendering** - All pieces now use the same Unicode symbols but with CSS styling to ensure they look uniform
4. **Black party**: Black icons (♚ ♛ ♜ ♝ ♞ ♟)
5. **White party**: White icons with black outline (♔ ♕ ♖ ♗ ♘ ♙)

### Screen Layout

The chess board shall be square an take the full width of the screen. On the top shall be a status indication which states if the program is still connected to the MQTT Broker. On disconnect or on start of the program it shall automatically try to connect.

A button below the chess board states "Neu starten" it will reset the board to the start position. White is on the bottom, black on the top.

