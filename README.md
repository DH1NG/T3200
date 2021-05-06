# T3200 - Zurrkraftdetektion an Spanngurten

Ein auf einem Raspberry Pi Pico basierendes Detektionssystem soll die Zurrkraft an Spanngurten detektieren. Dafür werden Piezo Sensoren und Aktoren verwendet. Das Sensorprinzip basiert auf SAW (Surface Acoustic Waves). Über die Dämpfung des übermittelten Ultraschalls kann die Dämpfung detektiert werden.

Hierfür wird benötigt:
  - Verstärkungselektronik für Piezosensor
  - Messbrücke für DMS
  - Ausgangsverstärkung des PIO Pins mittels Darlington Paar
  - SSD1306 OLED an i2c0
