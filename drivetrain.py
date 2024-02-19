#
# Copyright (c) Berk Akinci, FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds
from swervemodule import SwerveModule
from phoenix6.hardware import Pigeon2

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.tau/2  # 1/2 rotation per second
kModuleLocationComponent = 0.381
swerveConfig = {
    'swerveDrive' : { 'imu'      : { 'id'     :   14, }, },
    'frontRight'  : { 'drive'    : { 'id'     :    5, },
                      'angle'    : { 'id'     :    1, },
                      'encoder'  : { 'id'     :    9,
                                     'offset' : -135, }, # FIXME: placeholder
                      'location' : (+1, -1), },
    'backRight'   : { 'drive'    : { 'id'     :    8, },
                      'angle'    : { 'id'     :    4, },
                      'encoder'  : { 'id'     :   12,
                                     'offset' :  135, }, # FIXME: placeholder
                      'location' : (-1, -1), },
    'frontLeft'   : { 'drive'    : { 'id'     :    6, },
                      'angle'    : { 'id'     :    2, },
                      'encoder'  : { 'id'     :   10,
                                     'offset' :  -45, }, # FIXME: placeholder
                      'location' : (+1, +1), },
    'backLeft'    : { 'drive'    : { 'id'     :    7, },
                      'angle'    : { 'id'     :    3, },
                      'encoder'  : { 'id'     :   11,
                                     'offset' :   45, }, # FIXME: placeholder
                      'location' : (-1, +1), },
}

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """
    moduleOrder = ('frontLeft', 'frontRight', 'backLeft', 'backRight') # Must match wpimath API.

    def __init__(self) -> None:
        # Create SwerverModule objects from swerveConfig.
        modules = {}
        for (modName, modConfig) in swerveConfig.items():
            if modName == 'swerveDrive':
                continue
            newModule = {}
            translation = Translation2d(*modConfig['location'])
            translation *= kModuleLocationComponent
            newModule['location'] = translation
            newModule['swerveModule'] = SwerveModule(modConfig['drive']['id'],
                                                     modConfig['angle']['id'],
                                                     modConfig['encoder']['id'],
                                                     modConfig['encoder']['offset'])
            modules[modName] = newModule
        self.modules = modules

        self.gyro = Pigeon2(swerveConfig['swerveDrive']['imu']['id'])

        moduleLocations = [ modules[modName]['location']
                            for modName in Drivetrain.moduleOrder ]
        self.kinematics = SwerveDrive4Kinematics(*moduleLocations)

        self.odometry = SwerveDrive4Odometry(self.kinematics,
                                             Rotation2d.fromDegrees(self.gyro.get_yaw().value),
                                             self._swerveModuleGetPositions())

        self.gyro.set_yaw(0) # FIXME: shouldn't we reset before above objects?

    def _swerveModuleGetPositions(self):
        """Returns a tuple of SwerveModulePosition for all swerve modules."""
        moduleGetPositions = [ self.modules[modName]['swerveModule'].getPosition()
                               for modName in Drivetrain.moduleOrder ]
        return tuple(moduleGetPositions)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if fieldRelative:
            desiredChassisSpeeds = (
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                      Rotation2d.fromDegrees(self.gyro.get_yaw().value)) )
        else:
            desiredChassisSpeeds = (
                ChassisSpeeds(xSpeed, ySpeed, rot) )
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(desiredChassisSpeeds,
                                     periodSeconds) )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed)
        for idx, modName in enumerate(Drivetrain.moduleOrder):
            self.modules[modName]['swerveModule'].setDesiredState(swerveModuleStates[idx])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(Rotation2d.fromDegrees(self.gyro.get_yaw().value),
                             self._swerveModuleGetPositions())

    def debugSensorDump(self):
        dump = { modName : self.modules[modName]['swerveModule'].debugSensorDump()
                 for modName in Drivetrain.moduleOrder }
        dump['gyro'] = {
            'yaw' : self.gyro.get_yaw().value,
        }
        return dump
