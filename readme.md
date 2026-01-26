## Summary

Remote Chess allows two board players to compete online via a MQTT Broker. Lifting and placing of figures is detected by hall-sensors underneath the squares. The moves are published in an MQTT-topic called /remotechess. Boards subscribed to the topic get the moves from the topic and indicate them with NeoPixel LEDs which are integrated in the squares. Lifting a figure is indicated in red, placing a figure is indicated in green.

## Directory Structure



| Files / Directory                                            | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [FSD Remote Chess.md](https://github.com/JonahPi/RemoteChess/blob/main/FSD%20Remote%20Chess.md) | Function Specification Document                              |
| CAD                                                          | CAD STEP File for Chess Board with electronic and cover      |
| docs                                                         | Files used for Progressive Web App (PWA)2                    |
| Electronics                                                  | PCB schematic and Fritzing Files, Gerber Files               |
| Micropython                                                  | ESP32 Micropython Program, Secrets File sample, Board-test-script, State-Diagram |
| pics                                                         | Project pictures                                             |

