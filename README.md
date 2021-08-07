# SNES controller to N64 adapter

I'm a big fan of the "Bust-A-Move" series. I've played the SNES game with my "Score Master" arcade stick. I already had that at hand and it provides an "arcade-like" feeling while playing this classic arcade title. The successor titles "Bust-A-Move 2" and "Bust-A-Move 3" both were published for the N64 which either meant I had to use the regular N64 controller, get a new arcade stick or (and that's why this project came to live) need some adapter to use my "Score Master" on my N64.

This project heavily bases on the work done by @brownan in his [Gamecube-N64 Controller](https://github.com/brownan/Gamecube-N64-Controller) project. Without his assembler work, this project wouldn't have been possible!

# Materials

This is an extensive list of what's required.

- Any SNES controller you want to use on your N64
- An Arduino with an Atmega328 chip running at 16 MHz clock speed
- Gamecube and SNES extension cables to take the connectors from

# Quick setup

##  Hooking the N64 to the Arduino

The N64 controller cord had 3 wires: ground, +3.3V, and data. The pin-out is shown in Figure 1 (left).

1. +3.3V (red) - connect to nothing
2. Data (white) - connect to Arduino digital I/O 8
3. GND (black) - connect to Arduino ground

## Hooking up the SNES controller to the Arduino

The SNES controller has 5 wires

1. #5V - connect to Arduino +5V supply
2. Latch - connect to Arduino digital I/O A1
3. Clock - connect to Arduino digital I/O A2
4. Data - connect to Arduino digital I/O A3

## Using it

You can upload the project to your Arduino using the Arduino IDE. Then connect the Arduino to both, your SNES controller and your N64 and provide power to the Arduino using the USB port.

## Final product

I decided to use the Arduino Nano because it is very small and can be placed between the two connectors without taking up too much space. I also decided to use a step up converter to step up the 3.3 V from the N64 to 5 V to provide the required power to the Arduino and SNES controller. It takes around 40 mA of current from the N64 in this setup which I guessed was in a safe range to power directly from the N64 power supply.
