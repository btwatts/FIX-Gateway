# coding: utf8
#!/usr/bin/env python

#  Copyright (c) 2024 Byron Watts
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

#  Note:
#  This file serves as a starting point for a plugin.  This is a Thread based
#  plugin where the main Plugin class creates a thread and starts the thread
#  when the plugin's run() function is called.

import threading
import time
from collections import OrderedDict
import fixgw.plugin as plugin

from bmp388 import BMP388 as BMP388
import IMU.IMU as IMU

def handle_ctrl_c(signal, frame):
    print(" ")
    print(" Control-C ")
    print(" ulp plugin detected Control-C and is attempting to exit ")
    print(" ")
    sys.exit(130) # 130 is standard exit code for ctrl-c

class MainThread(threading.Thread):
    def __init__(self, parent):
        """The calling object should pass itself as the parent.
           This gives the thread all the plugin goodies that the
           parent has."""
        super(MainThread, self).__init__()
        print("running ulp plugin")
        self.getout = False   # indicator for when to stop
        self.parent = parent  # parent plugin object
        self.log = parent.log  # simplifies logging
        self.count = 0
        self.sleep_time = .03   # 3 x .03  gives +/-10Hz refresh rate
      # self.sleep_time = 0.005 # 3 x .005 gives +/-60Hz refresh rate

    def run(self):

        while not self.getout:
            time.sleep(self.sleep_time)
            self.count += 1
            self.log.debug("Yep")  # Do something more useful here
        self.running = False

    def stop(self):
        self.getout = True


class Plugin(plugin.PluginBase):
    """ All plugins for FIX Gateway should implement at least the class
        named 'Plugin.'  They should be derived from the base class in
        the plugin module.

        The run and stop methods of the plugin should be overridden but the
        base module functions should be called first."""
    def __init__(self, name, config):
        super(Plugin, self).__init__(name, config)
        self.thread = MainThread(self)
        self.status = OrderedDict()

        """Test initialization of multiple sensor packs."""
        self.bmp388 = BMP388()
        self.imu = IMU()

    def run(self):
        """ The run method should return immediately.  The main routine will
            block when calling this function.  If the plugin is simply a collection
            of callback functions, those can be setup here and no thread will be
            necessary"""
        super(Plugin, self).run()
        self.thread.start()

    def stop(self):
        """ The stop method should not return until the plugin has completely
            stopped.  This generally means a .join() on a thread.  It should
            also undo any callbacks that were set up in the run() method"""
        self.thread.stop()
        if self.thread.is_alive():
            self.thread.join(1.0)
        if self.thread.is_alive():
            raise plugin.PluginFail
        super(Plugin, self).stop()

    def is_running(self):
        return self.thread.is_alive()

    def get_status(self):
        """ The get_status method should return a dict or OrderedDict that
            is basically a key/value pair of statistics"""

        self.status["Alive"] = self.is_running(self) # interesting ?
        self.status["Count"] = self.thread.count     # I think
        return self.status

      #  return OrderedDict({"Count":self.thread.count})



    def test_data(self):
        """ Data copied here for reference from stratux test plugin
            and most likely will be deleted later"""

        getSituation = """{{
          "GPSLastFixSinceMidnightUTC": 67337.6,
          "GPSLatitude": {LAT},
          "GPSLongitude": {LONG},
          "GPSFixQuality": 4,
          "GPSHeightAboveEllipsoid": 115.51,
          "GPSGeoidSep": -17.523,
          "GPSSatellites": 5,
          "GPSSatellitesTracked": 11,
          "GPSSatellitesSeen": 8,
          "GPSHorizontalAccuracy": 10.2,
          "GPSNACp": 9,
          "GPSAltitudeMSL": 170.10767,
          "GPSVerticalAccuracy": 8,
          "GPSVerticalSpeed": -0.6135171,
          "GPSLastFixLocalTime": "0001-01-01T00:06:44.24Z",
          "GPSTrueCourse": 0,
          "GPSTurnRate": 0,
          "GPSGroundSpeed": 0.77598433056951,
          "GPSLastGroundTrackTime": "0001-01-01T00:06:44.24Z",
          "GPSTime": "2017-09-26T18:42:17Z",
          "GPSLastGPSTimeStratuxTime": "0001-01-01T00:06:43.65Z",
          "GPSLastValidNMEAMessageTime": "0001-01-01T00:06:44.24Z",
          "GPSLastValidNMEAMessage": "$PUBX,04,184426.00,260917,240266.00,1968,18,-177618,-952.368,21*1A",
          "GPSPositionSampleRate": 0,
          "BaroTemperature": 37.02,
          "BaroPressureAltitude": 153.32,
          "BaroVerticalSpeed": 1.3123479,
          "BaroLastMeasurementTime": "0001-01-01T00:06:44.23Z",
          "AHRSPitch": {PITCH},
          "AHRSRoll": {ROLL},
          "AHRSGyroHeading": 187741.08073052,
          "AHRSMagHeading": 3276.7,
          "AHRSSlipSkid": 0.52267604604907,
          "AHRSTurnRate": 3276.7,
          "AHRSGLoad": 0.99847599584255,
          "AHRSGLoadMin": 0.99815989027411,
          "AHRSGLoadMax": 1.0043409597397,
          "AHRSLastAttitudeTime": "0001-01-01T00:06:44.28Z",
          "AHRSStatus": 7
        }}"""
