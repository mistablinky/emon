# emon
This project realises the energy monitoring of an off-grid 500Wp solar system located in Austria. An ESP8266 board measures voltage and current of the connected solar panels and sends the data every 10 seconds to the EmonCMS cloud. [>>> Solar harvesting live data](https://emoncms.org/app/view?name=Solarernte&readkey=8b38e7dcc457e170a6cc96cb234fc605)

ESP8266 on a proto board with all sensors connected: ![ESP8266 on a proto board with all sensors connected](/images/IMG_7960.JPG)

**General information**
- OpenEnergyMonitor-Project: https://openenergymonitor.org/
- EmonCMS web-app: https://emoncms.org/site/home
- ESP8266-Board: Adafruit Feather HUZZAH (https://www.adafruit.com/product/2821)

**Used sensors**
- MCP3008 8-Channel 10-Bit A/D Converter with SPI interface (https://www.adafruit.com/product/856)
- ACS711EX Current Sensor Carrier -31A to +31A with analog output (https://www.pololu.com/product/2453)
- DS18B20 digital temperature sensor with Dallas 1-Wire interface (https://www.adafruit.com/product/374)

**Known issues**
- re-connect after wifi connection loss works not properly
- sporadic nonsense values from the DS18B20 temperature sensor
