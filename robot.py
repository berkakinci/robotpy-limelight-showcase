#!/usr/bin/env python
#
# Copyright (c) 2024 FIRST, Berk Akinci and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from limelight import LimeLight

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        print("Yes; we are alive")
        #print("Yup", NetworkTables.getTable("limelight").getNumber('tx', 0))
        ll=LimeLight()
        print("Yea", ll.getNumber('tx', 0))
        print("Yah", ll.tx)
        #print("Nah", ll.txvalid)
        print("Vectors", ll.getNumberArray('botpose'))
        print("VecYeah", ll.botpose)

    def robotPeriodic(self) -> None:
        pass

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
        pass
