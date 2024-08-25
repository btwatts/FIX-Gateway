
# bmp280.py
# # Note: This is a skeleton wrapper around the AdaFruit BMP280 library.

import board
import adafruit_bmp280

class BMP280:
  def __init__(self):
    self.i2c = board.I2C()
    self.sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)

  # Return temperature in Celcius....
  def get_temperature(self):
    return self.get_celcius()

  # Return temperature in Celcius
  def get_celcius(self):
    return self.sensor.temperature

  # Return temperature in Fahrenheit
  def get_fahrenheit(self):
    return 9/5*self.sensor.temperature + 32

  # Return pressure in hPa
  def get_pressure(self):
    return self.sensor.pressure

  # Return barametric based altitude in meters
  def get_altitude(self, seaLevelhPa = 1013.25):
    altitude = 44330 * (1.0 - pow(self.sensor.pressure / seaLevelhPa, 0.1902949571836346)) #* 100 # Note:  0.1902949571836346 is: 1/5.255
    return altitude

  # Return barametric based altitude in feet
  def get_altitude_in_feet(self, seaLevelhPa = 1013.25):
    altitude = self.get_altitude(seaLevelhPa)
    return altitude * 3.28084

  def get_temperature_and_pressure_and_altitude(self):
    """Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa."""
    temperature = self.get_temperature()
    pressure = self.get_pressure()
    altitude = self.get_altitude()
    return (temperature*100, pressure*100, altitude*100)

if __name__ == '__main__':

 import time
 
 print("BMP280 Test Program ...\n")
 
 bmp280 = BMP280()
 seaLevelhPa=1013.25

 for x in range(0,2): #while True:
  time.sleep(0.5)

  temperature = bmp280.get_temperature()
  print(f"temp C: {temperature}")
  print(f"temp F: {9/5*temperature + 32}")
  pressure = bmp280.get_pressure()
  print(f"press: {pressure}")

  newAlt = 44330*(1.0 - pow(pressure / seaLevelhPa, 0.1902949571836346)) #* 100 # Note:  0.1902949571836346 is: 1/5.255
  print(f"new alt: {newAlt}")
  print(f"check alt: {bmp280.get_altitude()}")

  print(f"altitude in feet: {bmp280.get_altitude_in_feet()}")

  print("-----")
  for x in range(0,2): # while True:
    time.sleep(0.5)
    temperature,pressure,altitude = bmp280.get_temperature_and_pressure_and_altitude()
    print(' Temperature = %.1f Pressure = %.2f  Altitude = %.2f '%(temperature/100.0,pressure/100.0,altitude/100.0))
