# Hardware Setup

## Stepper Control

Arduino Uno + DRV8825

Connections

STEP → Arduino D2  
DIR → Arduino D3  
ENABLE → Arduino D4  

Motor power supply

12–24 V recommended.

Set current limit before connecting motor.

---

## Temperature Measurement

MAX31865 SPI wiring

SCK → Raspberry Pi SCK  
MOSI → MOSI  
MISO → MISO  
CS → GPIO5

---

## Peltier PWM

Use a MOSFET driver.

PWM pin: GPIO18
