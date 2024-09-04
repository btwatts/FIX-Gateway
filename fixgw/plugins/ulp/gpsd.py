# Ref: https://gpsd.gitlab.io/gpsd/index.html
# Ref: https://gpsd.gitlab.io/gpsd/gpsd_json.html

from gps import *
from tzfpy import get_tz
from dateutil import tz
from datetime import datetime

class gps_mod:
  def __init__(self):
     self.gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

  def getLatLon(self):
      counter = 10 # limit the attempts in case the GPS has not yet fixed
      while counter > 0:
          counter -= 1
          nx = self.gpsd.next()
          #print(nx['class'])
          latitude = "Unknown"
          longitude = "Unkonwn"
          if nx['class'] == 'TPV':
              latitude  = getattr(nx,'lat', latitude)
              longitude = getattr(nx,'lon', longitude)
              break # BUGBUG does this do the same thing as set and return ??
              # ret = {'lon':longitude,'lat':latitude}
              # return ret
      ret = {'lon':longitude,'lat':latitude}
      return ret

  def getPositionData(self, tzone='America/Chicago'):
      nx = self.gpsd.next()

    # nx['class'] == 'VERSION':
    #  <dictwrapper: {'class': 'VERSION', 'release': '3.22', 'rev': '3.22', 'proto_major': 3, 'proto_minor': 14}>
    #
    # nx['class'] == 'DEVICES':
    #  <dictwrapper: {'class': 'DEVICES', 'devices': [{'class': 'DEVICE', 'path': '/dev/serial0', 'driver': 'u-blox', 'subtype': 'SW ROM CORE 3.01 (107888),HW 00080000', 'subtype1': 'FWVER=SPG 3.01,PROTVER=18.00,GPS;GLO;GAL;BDS,SBAS;IMES;QZSS', 'activated': '2024-09-02T12:03:44.590Z', 'flags': 1, 'native': 1, 'bps': 9600, 'parity': 'N', 'stopbits': 1, 'cycle': 1.0, 'mincycle': 0.02}]}>
    #
    # nx['class'] == 'WATCH':
    #  <dictwrapper: {'class': 'WATCH',   'enable': True, 'json': True, 'nmea': False, 'raw': 0, 'scaled': False, 'timing': False, 'split24': False, 'pps': False}>
    #
    # nx['class'] == 'SKY':
    #  <dictwrapper: {'class': 'SKY',     'device': '/dev/serial0', 'time': '2024-09-02T12:10:05.000Z', 'xdop': 0.82, 'ydop': 0.62, 'vdop': 1.99, 'tdop': 1.23, 'hdop': 1.03, 'gdop': 2.55, 'pdop': 2.24, 'nSat': 25, 'uSat': 8, 'satellites': [<dictwrapper: {'PRN': 3, 'el': 12.0, 'az': 285.0, 'ss': 0.0, 'used': False, 'gnssid': 0, 'svid': 3, 'health': 1}>, <dictwrapper: {'PRN': 4, 'el': 4.0, 'az': 325.0, 'ss': 0.0, 'used': False, 'gnssid': 0, 'svid': 4, 'health': 1}>, <dictwrapper: {'PRN': 8, 'el': 19.0, 'az': 210.0, 'ss': 27.0, 'used': True, 'gnssid': 0, 'svid': 8, 'health': 1}>, <dictwrapper: {'PRN': 10, 'el': 9.0, 'az': 165.0, 'ss': 16.0, 'used': True, 'gnssid': 0, 'svid': 10, 'health': 1}>, <dictwrapper: {'PRN': 16, 'el': 47.0, 'az': 301.0, 'ss': 27.0, 'used': True, 'gnssid': 0, 'svid': 16, 'health': 1}>, <dictwrapper: {'PRN': 18, 'el': 8.0, 'az': 91.0, 'ss': 0.0, 'used': False, 'gnssid': 0, 'svid': 18, 'health': 1}>, <dictwrapper: {'PRN': 26, 'el': 41.0, 'az': 353.0, 'ss': 0.0, 'used': False, 'gnssid': 0, 'svid': 26, 'health': 1}>, <dictwrapper: {'PRN': 27, 'el': 50.0, 'az': 198.0, 'ss': 28.0, 'used': True, 'gnssid': 0, 'svid': 27, 'health': 1}>, <dictwrapper: {'PRN': 28, 'el': 35.0, 'az': 51.0, 'ss': 28.0, 'used': True, 'gnssid': 0, 'svid': 28, 'health': 1}>, <dictwrapper: {'PRN': 29, 'el': 1.0, 'az': 37.0, 'ss': 0.0, 'used': False, 'gnssid': 0, 'svid': 29, 'health': 1}>, <dictwrapper: {'PRN': 31, 'el': 36.0, 'az': 8.0, 'ss': 21.0, 'used': True, 'gnssid': 0, 'svid': 31, 'health': 1}>, <dictwrapper: {'PRN': 32, 'el': 46.0, 'az': 145.0, 'ss': 25.0, 'used': True, 'gnssid': 0, 'svid': 32, 'health': 1}>, <dictwrapper: {'PRN': 40, 'el': 11.0, 'az': 267.0, 'ss': 0.0, 'used': False, 'gnssid': 1, 'svid': 127, 'health': 1}>, <dictwrapper: {'PRN': 41, 'el': 41.0, 'az': 261.0, 'ss': 0.0, 'used': False, 'gnssid': 1, 'svid': 128}>, <dictwrapper: {'PRN': 42, 'el': 70.0, 'az': 118.0, 'ss': 0.0, 'used': False, 'gnssid': 1, 'svid': 129}>, <dictwrapper: {'PRN': 50, 'el': 65.0, 'az': 111.0, 'ss': 0.0, 'used': False, 'gnssid': 1, 'svid': 137}>, <dictwrapper: {'PRN': 67, 'el': 44.0, 'az': 181.0, 'ss': 19.0, 'used': False, 'gnssid': 6, 'svid': 3, 'health': 1}>, <dictwrapper: {'PRN': 68, 'el': 65.0, 'az': 280.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 4, 'health': 1}>, <dictwrapper: {'PRN': 69, 'el': 22.0, 'az': 325.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 5, 'health': 1}>, <dictwrapper: {'PRN': 78, 'el': 18.0, 'az': 12.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 14, 'health': 1}>, <dictwrapper: {'PRN': 79, 'el': 22.0, 'az': 319.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 15, 'health': 1}>, <dictwrapper: {'PRN': 80, 'el': 3.0, 'az': 272.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 16, 'health': 1}>, <dictwrapper: {'PRN': 81, 'el': 31.0, 'az': 137.0, 'ss': 16.0, 'used': True, 'gnssid': 6, 'svid': 17, 'health': 1}>, <dictwrapper: {'PRN': 82, 'el': 9.0, 'az': 181.0, 'ss': 0.0, 'used': False, 'gnssid': 6, 'svid': 18, 'health': 1}>, <dictwrapper: {'PRN': 88, 'el': 24.0, 'az': 76.0, 'ss': 22.0, 'used': False, 'gnssid': 6, 'svid': 24, 'health': 1}>]}>
    #
    # nx['class'] == 'TPV':
    #  <dictwrapper: {'class': 'TPV',     'device': '/dev/serial0', 'mode': 3, 'time': '2024-09-02T12:11:57.000Z', 'leapseconds': 18, 'ept': 0.005, 'lat': 8.1629034, 'lon': 125.1225247, 'altHAE': 734.707, 'altMSL': 676.89, 'alt': 676.89, 'epv': 13.01, 'track': 319.3477, 'magtrack': 318.1374, 'magvar': -1.2, 'speed': 0.164, 'eps': 0.87, 'ecefx': -3632998.6, 'ecefy': 5164917.41, 'ecefz': 899722.17, 'ecefvx': -0.08, 'ecefvy': -0.11, 'ecefvz': -0.07, 'ecefpAcc': 15.09, 'ecefvAcc': 0.89, 'geoidSep': 63.554, 'eph': 7.552, 'sep': 41.99}>
    #
    #  time   -- UTC
    #  leapseconds
    #  ept
    #  lat
    #  lon
    #  altHAE
    #  altMSL
    #  alt
    #  epv
    #  track
    #  magtrack
    #  magvar
    #  speed
    #  eps
    #  ecefx
    #  ecefy
    #  ecefz
    #  ecefpAcc
    #  ecefvAcc
    #  geoidSep
    #  eph
    #  sep

      if nx['class'] == 'TPV':
          latitude = getattr(nx,'lat', "Unknown")
          longitude = getattr(nx,'lon', "Unknown")
          speed = getattr(nx,'speed', "Unknown")
          timestring = getattr(nx,'time', "Unknown")

          utc_datetime = datetime.strptime(timestring, '%Y-%m-%dT%H:%M:%S.%fZ') # 2024-09-02T14:47:39.000Z
          from_zone = tz.gettz('UTC')
          to_zone = tz.gettz(tzone)
          utc_datetime = utc_datetime.replace(tzinfo=from_zone)
          time = utc_datetime.astimezone(to_zone)
          print(f"Your position: lon: {longitude}  lat: {latitude}  speed: {speed}  time: {time}") # tz: {time.tzinfo}")
          #print(f"timestring: {timestring}")


if __name__ == '__main__':

  def runTest(self):

      import time

      self.running = True

      try:
          print("Application started!")
          value = self.getLatLon()
          tzone = get_tz(value['lon'], value['lat'])
          print(f"Local timezone: {tzone}")
          while self.running:
              self.getPositionData(tzone)
              time.sleep(1.0)

      except (KeyboardInterrupt):
          self.running = False
          print("Applications closed!")

# At the moment I'm having trouble with test and I'm deferring it until later
# gpstest = gps_mod()
# gpstest.runTest()
