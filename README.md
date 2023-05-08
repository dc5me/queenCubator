# queenCubator
Raise queen bees in a temperature-controlled incubator
- Measure the temperature in the incubator
- Control the temperature with power resistors via PID controller
- Log the temperature and humidity according to thinkspeak.com

Hardware:
- ESP8266-12F W (D1 mini)
- BME280 (for logging to thingspeak)
- DS18B20 (for temperature control)
- 7 Ceramic resistors (1R 5W) as floor heater
- MOSFET for PWM controlled regulation of the heating resistors
- Clay for embedding the resistors and regulating the air humidity
- A glass of water for passive humidity (~70%)
- As housing any insulating material can be used. e.g. 20mm alu-laminated plates from BauderPIR
