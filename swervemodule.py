#
# Copyright (c) Berk Akinci, FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
import rev
import phoenix5.sensors

kWheelRadius = 0.0508
kDriveGearRatio = 8.14/1
kTurningEncoderDirectionCW = True
kTurningEncoderCodeCount = 2**12
kTurningEncoderTimeoutMS = 500
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        turningEncoderID: int,
        turningEncoderOffsetDegrees: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor and turning encoder.

        :param driveMotorID:      CAN ID the drive motor.
        :param turningMotorID:    CAN ID for the turning motor.
        :param turningEncoderID:  CAN ID for the turning encoder
        """
        self.driveMotor = rev.CANSparkMax(driveMotorID)
        self.turningMotor = rev.CANSparkMax(turningMotorID)

        self.driveEncoder = driveMotor.getEncoder()
        self.turningEncoder = phoenix5.sensors.CANCoder(turningEncoderID)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = PIDController(0.01, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = ProfiledPIDControllerRadians(
            0.01,
            0,
            0,
            TrapezoidProfile.Constraints(
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

        # Set the conversion factor to get radians out of encoder.
        # wpimath.geometry.Rotation2d() wants radians.
        self.turningEncoder.configAbsoluteSensorRange(phoenix5.sensors.AbsoluteSensorRange.Unsigned_0_to_360,
                                                      kTurningEncoderTimeoutMS)
        self.turningEncoder.configSensorDirection(kTurningEncoderDirectionCW,
                                                  kTurningEncoderTimeoutMS)
        self.turningEncoder.configMagnetOffset(offsetDegrees = turningEncoderOffsetDegrees,
                                               timeoutMs = kTurningEncoderTimeoutMS)
        self.turningEncoder.configFeedbackCoefficient(sensorCoefficient = math.tau/kTurningEncoderCodeCount,
                                                      unitString = 'radians',
                                                      timeoutMs = kTurningEncoderTimeoutMS)

        # Limit the PID Controller's input range between 0 and tau and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(0, math.tau)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getAbsolutePosition()), # FIXME: Verify 0-tau absolute is OK.  otherwise getPosition()
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return SwerveModulePosition(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getAbsolutePosition()), # FIXME: this or getPosition()
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
