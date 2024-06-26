#!/usr/bin/python
#
#    This program  reads the angles from the acceleromteer, gyroscope
#    and mangnetometer on a BerryIMU connected to a Raspberry Pi.
#
#    This program includes two filters (low pass and median) to improve the
#    values returned from BerryIMU by reducing noise.
#
#    The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
#
#    This script is python 2.7 and 3 compatible
#
#    Feel free to do whatever you like with this code.
#    Distributed as-is; no warranty is given.
#
#    http://ozzmaker.com/



import sys
import math
from fixgw.plugins.ulp.IMU.IMU import IMU
import os


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40              # Complementary filter constant
MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay

class BERRYIMU(object):

    def __init__(self):
        self.imu = IMU()
################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading value.

        self.magXmin =  -562
        self.magYmin =  -1002
        self.magZmin =  1811
        self.magXmax =  3096
        self.magYmax =  3261
        self.magZmax =  6458

############### END Calibration offsets #################


    #Kalman filter variables
        self.Q_angle = 0.02
        self.Q_gyro = 0.0015
        self.R_angle = 0.005
        self.y_bias = 0.0
        self.x_bias = 0.0
        self.XP_00 = 0.0
        self.XP_01 = 0.0
        self.XP_10 = 0.0
        self.XP_11 = 0.0
        self.YP_00 = 0.0
        self.YP_01 = 0.0
        self.YP_10 = 0.0
        self.YP_11 = 0.0
        self.KFangleX = 0.0
        self.KFangleY = 0.0

    # Set default values for processing
        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.CFangleXFiltered = 0.0
        self.CFangleYFiltered = 0.0
        self.kalmanX = 0.0
        self.kalmanY = 0.0
        self.oldXMagRawValue = 0
        self.oldYMagRawValue = 0
        self.oldZMagRawValue = 0
        self.oldXAccRawValue = 0
        self.oldYAccRawValue = 0
        self.oldZAccRawValue = 0

    #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
        self.acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
        self.mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

    def version(self):
        return self.imu.version()

    def kalmanFilterY (self, accAngle, gyroRate, DT):
        self.y=0.0
        self.S=0.0

        self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)
    
        self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
        self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
        self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
        self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )
    
        self.y = accAngle - self.KFangleY
        self.S = self.YP_00 + self.R_angle
        self.K_0 = self.YP_00 / self.S
        self.K_1 = self.YP_10 / self.S
    
        self.KFangleY = self.KFangleY + ( self.K_0 * self.y )
        self.y_bias = self.y_bias + ( self.K_1 * self.y )
    
        self.YP_00 = self.YP_00 - ( self.K_0 * self.YP_00 )
        self.YP_01 = self.YP_01 - ( self.K_0 * self.YP_01 )
        self.YP_10 = self.YP_10 - ( self.K_1 * self.YP_00 )
        self.YP_11 = self.YP_11 - ( self.K_1 * self.YP_01 )
    
        return self.KFangleY
    
    def kalmanFilterX (self, accAngle, gyroRate, DT):
        self.x=0.0
        self.S=0.0
    
        self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)
    
        self.XP_00 = self.XP_00 + ( - DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT )
        self.XP_01 = self.XP_01 + ( - DT * self.XP_11 )
        self.XP_10 = self.XP_10 + ( - DT * self.XP_11 )
        self.XP_11 = self.XP_11 + ( + self.Q_gyro * DT )
    
        self.x = accAngle - self.KFangleX
        self.S = self.XP_00 + self.R_angle
        self.K_0 = self.XP_00 / self.S
        self.K_1 = self.XP_10 / self.S
    
        self.KFangleX = self.KFangleX + ( self.K_0 * self.x )
        self.x_bias = self.x_bias + ( self.K_1 * self.x )
    
        self.XP_00 = self.XP_00 - ( self.K_0 * self.XP_00 )
        self.XP_01 = self.XP_01 - ( self.K_0 * self.XP_01 )
        self.XP_10 = self.XP_10 - ( self.K_1 * self.XP_00 )
        self.XP_11 = self.XP_11 - ( self.K_1 * self.XP_01 )
    
        return self.KFangleX

    def initialize(self):
        self.imu.detectIMU()     #Detect if BerryIMU is connected.
        if(self.imu.version == 99):
            print(" No BerryIMU found... normally would be exiting here...")
            # sys.exit()
        self.imu.initIMU()       #Initialise the accelerometer, gyroscope and compass

    def readIMU(self):
        #Read the accelerometer,gyroscope and magnetometer values
        ACCx = self.imu.readACCx()
        ACCy = self.imu.readACCy()
        ACCz = self.imu.readACCz()
        GYRx = self.imu.readGYRx()
        GYRy = self.imu.readGYRy()
        GYRz = self.imu.readGYRz()
        MAGx = self.imu.readMAGx()
        MAGy = self.imu.readMAGy()
        MAGz = self.imu.readMAGz()

        return {'ACCx':ACCx, 'ACCy':ACCy, 'ACCz':ACCz, 'GYRx':GYRx, 'GYRy':GYRy, 'GYRz':GYRz, 'MAGx':MAGx, 'MAGy':MAGy, 'MAGz':MAGz}

    def readCalibrated(self, LP):
        imuDict = BERRYIMU.readIMU(self)
        ACCx = imuDict['ACCx']
        ACCy = imuDict['ACCy']
        ACCz = imuDict['ACCz']
        GYRx = imuDict['GYRx']
        GYRy = imuDict['GYRy']
        GYRz = imuDict['GYRz']
        MAGx = imuDict['MAGx']
        MAGy = imuDict['MAGy']
        MAGz = imuDict['MAGz']

        #Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) /2
        MAGy -= (self.magYmin + self.magYmax) /2
        MAGz -= (self.magZmin + self.magZmax) /2

    ###############################################
    #### Apply low pass filter ####
    ###############################################
        MAGx =  MAGx  * MAG_LPF_FACTOR + self.oldXMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGy =  MAGy  * MAG_LPF_FACTOR + self.oldYMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGz =  MAGz  * MAG_LPF_FACTOR + self.oldZMagRawValue*(1 - MAG_LPF_FACTOR);
        ACCx =  ACCx  * ACC_LPF_FACTOR + self.oldXAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCy =  ACCy  * ACC_LPF_FACTOR + self.oldYAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCz =  ACCz  * ACC_LPF_FACTOR + self.oldZAccRawValue*(1 - ACC_LPF_FACTOR);
    
        self.oldXMagRawValue = MAGx
        self.oldYMagRawValue = MAGy
        self.oldZMagRawValue = MAGz
        self.oldXAccRawValue = ACCx
        self.oldYAccRawValue = ACCy
        self.oldZAccRawValue = ACCz

    #########################################
    #### Median filter for accelerometer ####
    #########################################
        # cycle the table
        for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
            self.acc_medianTable1X[x] = self.acc_medianTable1X[x-1]
            self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x-1]
            self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x-1]

    # Insert the lates values
        self.acc_medianTable1X[0] = ACCx
        self.acc_medianTable1Y[0] = ACCy
        self.acc_medianTable1Z[0] = ACCz

    # Copy the tables
        self.acc_medianTable2X = self.acc_medianTable1X[:]
        self.acc_medianTable2Y = self.acc_medianTable1Y[:]
        self.acc_medianTable2Z = self.acc_medianTable1Z[:]

    # Sort table 2
        self.acc_medianTable2X.sort()
        self.acc_medianTable2Y.sort()
        self.acc_medianTable2Z.sort()

    # The middle value is the value we are interested in
        ACCx = self.acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
        ACCy = self.acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
        ACCz = self.acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];

    #########################################
    #### Median filter for magnetometer ####
    #########################################
    # cycle the table
        for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
            self.mag_medianTable1X[x] = self.mag_medianTable1X[x-1]
            self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x-1]
            self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x-1]

    # Insert the latest values
        self.mag_medianTable1X[0] = MAGx
        self.mag_medianTable1Y[0] = MAGy
        self.mag_medianTable1Z[0] = MAGz

    # Copy the tables
        self.mag_medianTable2X = self.mag_medianTable1X[:]
        self.mag_medianTable2Y = self.mag_medianTable1Y[:]
        self.mag_medianTable2Z = self.mag_medianTable1Z[:]

    # Sort table 2
        self.mag_medianTable2X.sort()
        self.mag_medianTable2Y.sort()
        self.mag_medianTable2Z.sort()

    # The middle value is the value we are interested in
        MAGx = self.mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
        MAGy = self.mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
        MAGz = self.mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];

    #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * G_GAIN
        rate_gyr_y =  GYRy * G_GAIN
        rate_gyr_z =  GYRz * G_GAIN


    #Calculate the angles from the gyro.
        self.gyroXangle+=rate_gyr_x*LP
        self.gyroYangle+=rate_gyr_y*LP
        self.gyroZangle+=rate_gyr_z*LP

    #Convert Accelerometer values to degrees
        AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
        AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG


    #Change the rotation value of the accelerometer to -/+ 180 and
    #move the Y axis '0' point to up.  This makes it easier to read.
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0

    #Complementary filter used to combine the accelerometer and gyro values.
        self.CFangleX=AA*(self.CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        self.CFangleY=AA*(self.CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

    #Kalman filter used to combine the accelerometer and gyro values.
        kalmanY = BERRYIMU.kalmanFilterY(self, AccYangle, rate_gyr_y, LP)
        kalmanX = BERRYIMU.kalmanFilterX(self, AccXangle, rate_gyr_x, LP)

    #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/M_PI

    #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    #Normalize accelerometer raw values.
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

    #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))

        return {'ACCx':ACCx, 'ACCy':ACCy, 'ACCz':ACCz, 'GYRx':GYRx, 'GYRy':GYRy, 'GYRz':GYRz, 'MAGx':MAGx, 'MAGy':MAGy, 'MAGz':MAGz,
                'rate_gyr_x':rate_gyr_x, 'rate_gyr_y':rate_gyr_y, 'rate_gyr_z':rate_gyr_z,
                'gyroXangle':self.gyroXangle, 'gyroYangle':self.gyroYangle, 'gyroZangle':self.gyroZangle,
                'AccXangle':AccXangle, 'AccYangle':AccYangle, 'CFangleX':self.CFangleX, 'CFangleY':self.CFangleY,
                'kalmanX':kalmanX, 'kalmanY':kalmanY,
                'heading':heading,
                'accXnorm':accXnorm, 'accYnorm':accYnorm,
                'pitch':pitch, 'roll':roll}



if __name__ == '__main__':

    def runTest(self):
        import time
        from datetime import datetime
        a = datetime.now()
        print("berryIMU Test Program ...\n")

        berryIMU = BERRYIMU()
        berryIMU.initialize()

        while True:

          ##Calculate loop Period(LP). How long between Gyro Reads
          b  = datetime.now() - a
          a  = datetime.now()
          LP = b.microseconds/(1000000*1.0)
          outputString = "Loop Time %5.2f " % ( LP )

          imuValues = berryIMU.readCalibrated(LP)

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
          else:                                                                #LSM9DS1
              magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

          #Y compensation
          if(self.imu.version() == 1 or self.imu.version() == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
              magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
          else:                                                                #LSM9DS1
              magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

          #Calculate tilt compensated heading
          tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

          if tiltCompensatedHeading < 0:
              tiltCompensatedHeading += 360

          ##################### END Tilt Compensation ########################

          print("\n")
          print("\n")
          print(outputString)

          if 1:                       #Change to '0' to stop showing the angles from the accelerometer
              outputString = "\n"
              outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)
              print(outputString)

          if 1:                       #Change to '0' to stop  showing the angles from the gyro
              outputString = "\n"
              outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)
              print(outpuString)

          if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
              outputString = "\n"
              outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)
              print(outputString)

          if 1:                       #Change to '0' to stop  showing the heading
              outputString = "\n"
              outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)
              print(outputString)

          if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
              outputString = "\n"
              outputString +="# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)
              print(outputString)

          #slow program down a bit, makes the output more readable
          time.sleep(2)

