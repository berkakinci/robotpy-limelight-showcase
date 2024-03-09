#!/usr/bin/env python
#
# Copyright (c) 2024 FIRST, Berk Akinci and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import math
from limelight import LimeLight
from drivetrain import Drivetrain

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.serial_number=Robot._get_platform_sn()
        print(f"Yes; we are alive: SN {self.serial_number}")

        self.counter=0
        self.ll=LimeLight()
        self.swerve=Drivetrain()

        self.ll.waitReady(verbose=True) # Will hang here.

        print("Yah", self.ll.camerapose_robotspace)
        #print("Nah", self.ll.txvalidbogusattribute) # Should raise
        print("Vector the long way", self.ll.getNumberArray('botpose'))
        print("Vector the easy way", self.ll.botpose)
        print("ledMode", self.ll.ledMode,
              "camMode", self.ll.camMode,
              "pipeline", self.ll.pipeline,
              "getpipe", self.ll.getpipe)
        self.ll.ledMode=(self.ll.ledMode+1)%4
        self.ll.camMode=0
        self.ll.pipeline=0
        print("ledMode", self.ll.ledMode,
              "camMode", self.ll.camMode,
              "pipeline", self.ll.pipeline,
              "getpipe", self.ll.getpipe)

    def robotPeriodic(self) -> None:
        self.counter+=1
        self.counter%=25
        if self.counter==0: # Every half second.
            print('tv', self.ll.tv,
                  'tx', self.ll.tx,
                  'ty', self.ll.ty,
                  'tid', self.ll.tid,
                  'targetpose_robotspace', self.ll.targetpose_robotspace)
            print(self.swerve.debugSensorDump())
        self.swerve.updateOdometry()

    def disabledPeriodic(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        self.drive()

    def teleopPeriodic(self) -> None:
        self.drive()

    def testPeriodic(self) -> None:
        pass

    def _simulationPeriodic(self) -> None:
        pass

    def drive(self) -> None:
        fieldRelative = False
        xSpeed = 0
        ySpeed = 0
        rotSpeed = math.tau/10
        self.swerve.drive(xSpeed, ySpeed, rotSpeed, fieldRelative, self.getPeriod())

    @staticmethod
    def _get_platform_sn() -> str:
        """
        Gets serial number of the platform we're running on.
        """
        sn=''
        try:
            with open('/sys/firmware/devicetree/base/serial-number') as snfile:
                sn=snfile.read()
                sn=sn.strip('\x00')
        except FileNotFoundError:
            sn='simulation'
        return sn
