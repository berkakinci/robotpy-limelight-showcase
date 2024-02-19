#
# Copyright (c) Berk Akinci, FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfileRadians
import rev
import phoenix6.hardware as ctre
import phoenix6.configs as ctre_configs

kWheelRadius = 0.0508
kDriveGearRatio = 8.14/1
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class radCANcoder(ctre.CANcoder):
    """
    Derived from CANcoder to add radian accessors.
    wpimath.geometry.Rotation2d() wants radians.
    """
    def get_absolute_position_radians(self):
        return math.tau * self.get_absolute_position().value

    def get_position_radians(self):
        return math.tau * self.get_position().value


class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        turningEncoderID: int,
        turningEncoderOffsetRotations: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor and turning encoder.

        :param driveMotorID:      CAN ID the drive motor.
        :param turningMotorID:    CAN ID for the turning motor.
        :param turningEncoderID:  CAN ID for the turning encoder
        :param turningEncoderOffsetRotations: Encoder offset (in rotations) for pointing to robot front (+X)
        """
        self.driveMotor = rev.CANSparkMax(driveMotorID,
                                          rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorID,
                                            rev.CANSparkLowLevel.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = radCANcoder(turningEncoderID)
        self.turningMotorEncoder = self.turningMotor.getEncoder() # For Debug only

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = PIDController(0.01, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = ProfiledPIDControllerRadians(
            0.01,
            0,
            0,
            TrapezoidProfileRadians.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = SimpleMotorFeedforwardMeters(0.01, 3)
        self.turnFeedforward = SimpleMotorFeedforwardMeters(0.01, 0.5)

        # Set the conversion factor to get meters/second out of encoder.
        # wpimath.kinematics.SwerveModuleState wants units m/s
        # Native units for encoder are RPM.
        # We use the distance traveled for one rotation of the wheel, gear ratio and
        # conversion from the minutes to seconds.
        wheelCircumference = math.tau * kWheelRadius
        motorRevolutionsPerMeter = kDriveGearRatio * (1 / wheelCircumference)
        self.driveEncoder.setVelocityConversionFactor(
            60 / motorRevolutionsPerMeter
        )

        # Set up encoder for our PID input expectations; and robot encoder magnet offsets.
        turningEncoderConfig = ctre_configs.CANcoderConfiguration()
        magSensor = turningEncoderConfig.magnet_sensor
        magSensor.with_sensor_direction(magSensor.sensor_direction.CLOCKWISE_POSITIVE)
        magSensor.with_absolute_sensor_range(magSensor.absolute_sensor_range.SIGNED_PLUS_MINUS_HALF)
        magSensor.with_magnet_offset(turningEncoderOffsetRotations)
        self.turningEncoder.configurator.apply(turningEncoderConfig)

        # Limit the PID Controller's input range between -tau/2 and +tau/2 and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.tau/2, math.tau/2)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.get_absolute_position_radians()), # FIXME: Verify -tau/2 - tau/2 absolute is OK.  otherwise get_position()
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return SwerveModulePosition(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.get_absolute_position_radians()), # FIXME: this or get_position()
        )

    def setDesiredState(
        self, desiredState: SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        currentState = self.getState()

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState,
                                                  currentState.angle)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        desiredState.speed *= (desiredState.angle - currentState.angle).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(currentState.speed,
                                                        desiredState.speed)
        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(currentState.angle.radians(),
                                                         desiredState.angle.radians())

        turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint().velocity)

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

    def debugSensorDump(self):
        """Returns dictionary with sensor readings for debug purposes."""
        dump={}
        dump['driveEncoderPos'] = self.driveEncoder.getPosition()
        dump['driveEncoderVel'] = self.driveEncoder.getVelocity()
        dump['turningEncoderAbsPos'] = self.turningEncoder.get_absolute_position().value
        dump['turningEncoderRelPos'] = self.turningEncoder.get_position().value
        dump['turningEncoderVel'] = self.turningEncoder.get_velocity().value
        dump['turningMotorEncoderPos'] = self.turningMotorEncoder.getPosition()
        dump['turningMotorEncoderVel'] = self.turningMotorEncoder.getVelocity()
        return dump
