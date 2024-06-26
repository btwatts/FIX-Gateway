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

import os
import sys
import threading
import math
import time
from datetime import datetime
from collections import OrderedDict
import fixgw.plugin as plugin

from fixgw.plugins.ulp.bmp388 import BMP388
from fixgw.plugins.ulp.IMU.berryIMU import BERRYIMU

def handle_ctrl_c(signal, frame):
    print(" ")
    print(" Control-C ")
    print(" ulp plugin detected Control-C and is attempting to exit ")
    print(" ")
    time.sleep(2)
    sys.exit(130) # 130 is standard exit code for ctrl-c

class MainThread(threading.Thread):
    def __init__(self, parent):
        """The calling object should pass itself as the parent.
           This gives the thread all the plugin goodies that the
           parent has."""
        super(MainThread, self).__init__()
        print("running ulp plugin")
        self.getout = False     # indicator for when to stop
        self.parent = parent    # parent plugin object
        self.log = parent.log   # simplifies logging

      # self.sleep_time = 3     # BUGBUG
        self.sleep_time = .03   # 3 x .03  gives +/-10Hz refresh rate
      # self.sleep_time = 0.005 # 3 x .005 gives +/-60Hz refresh rate

        self.count = 0
        self.alt = 0
        self.smooted = 0.8 # smooth altitude 0 to 1, 1 is very smooth.

        """Test initialization of multiple sensor packs."""
        self.bmp388 = BMP388()
        self.imu = BERRYIMU()

    def run(self):

        self.bmp388.initialize()
        self.imu.initialize()

        a = datetime.now()
        print("ulp plugin Test ... ")
        if 1:                       #Change to '0' to stop showing the current path
            print("  path  " + os.path.dirname(__file__))

        print ("\n")

        while not self.getout:
            time.sleep(self.sleep_time)
            self.count += 1

            temperature,pressure,altitude = self.bmp388.get_temperature_and_pressure_and_altitude()
            ## print('  Temperature = %.1f Pressure = %.2f  Altitude =%.2f '%(temperature/100.0,pressure/100.0,altitude/100.0))

            ##Calculate loop Period(LP). How long between Gyro Reads
            b  = datetime.now() - a
            a  = datetime.now()
            LP = b.microseconds/(1000000*1.0)
            outputString = "  Loop Time %5.2f " % ( LP )
        
            imuValues = self.imu.readCalibrated(LP)
        
            MAGx       = imuValues['MAGx']
            MAGy       = imuValues['MAGy']
            MAGz       = imuValues['MAGz']
            pitch      = imuValues['pitch']
            roll       = imuValues['roll']
            AccXangle  = imuValues['AccXangle']
            AccYangle  = imuValues['AccYangle']
            gyroXangle = imuValues['gyroXangle']
            gyroYangle = imuValues['gyroYangle']
            gyroZangle = imuValues['gyroZangle']
            CFangleX   = imuValues['CFangleX']
            CFangleY   = imuValues['CFangleY']
            heading    = imuValues['heading']
            kalmanX    = imuValues['kalmanX']
            kalmanY    = imuValues['kalmanY']

            #Calculate the new tilt compensated values
            #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
            #This needs to be taken into consideration when performing the calculations
        
            #X compensation
            if(self.imu.version() == 1 or self.imu.version() == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
                magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            else:                                                              #LSM9DS1
                magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)
        
            #Y compensation
            if(self.imu.version() == 1 or self.imu.version() == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
            else:                                                              #LSM9DS1
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)
        
            #Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/math.pi
        
            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360
        
            ##################### END Tilt Compensation ########################

            self.parent.db_write("PITCH", pitch)
            time.sleep(self.sleep_time)
            self.parent.db_write("ROLL", roll)
            time.sleep(self.sleep_time)
            self.parent.db_write("HEAD", heading)
            time.sleep(self.sleep_time)
        #   self.parent.db_write("ALAT", ) # based on 'slipskid' which is unknown to me right now
        #   time.sleep(self.sleep_time)
            self.parent.db_write("AIRPRESS", pressure/100.0)  # BUGBUG matching below
            time.sleep(self.sleep_time)
        # BUGBUG         currentbaro = self.parent.db_read("BARO")
        # BUGBUG         stdbaro = currentbaro[0]  # 29.92
            stdbaro = 29.92
            currentbaro = [] # BUGBUG DEBUG
            currentbaro.append(stdbaro) # BUGBUG DEBUG
        #    init_alt = round((float(altitude)*3.28083989502))
        #    self.alt = float((self.alt*self.smooted)+(1.0-self.smooted)*(init_alt))
        #    myAltitude = ((float(currentbaro[0]) - stdbaro)*1000) + self.alt
            self.parent.db_write("ALT", altitude/100) # myAltitude
            time.sleep(self.sleep_time)

            self.parent.db_write("BARO", pressure/100.0) # --- Altimiter setting ???
            time.sleep(self.sleep_time)
        #   self.parent.db_write("AOA", )  # AOA  - Angle of attack       ?
        #   self.parent.db_write("GS", )   # GS   - Ground speed          has to come from GPS
        #   self.parent.db_write("LAT", )  # LAT  - Latitude              has to come from GPS
        #   self.parent.db_wrtie("LONG", ) # LONG - Longitude             has to come from GPS
        #   self.parent.db_write("VS", )   # VS   - Vertical speed speed  has to come from GPS
        #   self.parent.db_write("IAS", )  # IAS  - Indicated airspeed    has to come from GPS

            print("\n")

            if 1:                       #Change to '0' to stop showing loop counter
                outputString += "  # " + str( self.count )

            print(outputString)  # this should print the loop time header whether or not the counter is included

            if 1:    # debug
                outputString  = "\n\t"
                outputString += '# Current BARO = %.2f'%(currentbaro[0])
                print(outputString)

            if 1:                       #Change to '0' to stop showing Temperature Pressure and Altitude
                outputString  = "\n\t"
                outputString += '# Temperature = %.1f Pressure = %.2f Altitude = %.2f'%(temperature/100.0,pressure/100.0,altitude/100)
          #      outputString += "\n\t"
          #      outputString += '# Altitude = %.2f  myAltitude = %.2f'%(altitude/100,myAltitude/100)
                print(outputString)

            if 1:                       #Change to '0' to stop showing the angles from the accelerometer
                outputString  = "\n\t"
                outputString += "# ACCX %5.2f ACCY %5.2f" % (AccXangle, AccYangle)
                print(outputString)
        
            if 1:                       #Change to '0' to stop  showing the angles from the gyro
                outputString  = "\n"
                outputString +="\t# GRYX %5.2f  GYRY %5.2f  GYRZ %5.2f" % (gyroXangle,gyroYangle,gyroZangle)
                print (outputString)
        
            if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
                outputString  = "\n"
                outputString +="\t# CFangleX %5.2f   CFangleY %5.2f" % (CFangleX,CFangleY)
                print(outputString)
        
            if 1:                       #Change to '0' to stop  showing the heading
                outputString  = "\n"
                outputString +="\t# HEADING %5.2f   tiltCompHeading %5.2f" % (heading,tiltCompensatedHeading)
                print(outputString)
        
            if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
                outputString  = "\n\t"
                outputString +="# kalmanX %5.2f kalmanY %5.2f" % (kalmanX,kalmanY)
                print(outputString)
            if 1:
                time.sleep(2) # BUGBUG debugging this way for now

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

    def run(self):
        """ The run method should return immediately.  The main routine will
            block when calling this function.  If the plugin is simply a collection
            of callback functions, those can be setup here and no thread will be
            necessary"""
        if (hasattr(super, "run") and callable(super.run)):
            print("calling super.run()")
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

        if (hasattr(super, "stop") and callable(super.stop)):
            print("calling super.stop()")
            super(Plugin, self).stop()
        print("Caught stop signal....next we exit.")
        time.sleep(2)
        os._exit(os.EX_OK)

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
