# Copyright (c) 2024 FIRST, Berk Akinci
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from networktables import NetworkTables

class LimeLight:
    _ll_attributes = {
        # Basic Targeting Data
        'tv' : 'Whether the limelight has any valid targets (0 or 1)',
        'tx' : 'Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)',
        'ty' : 'Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)',
        'ta' : 'Target Area (0% of image to 100% of image)',
        'tl' : 'The pipeline\'s latency contribution (ms). Add to "cl" to get total latency.,',
        'cl' : 'Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.',
        'tshort' : 'Sidelength of shortest side of the fitted bounding box (pixels)',
        'tlong' : 'Sidelength of longest side of the fitted bounding box (pixels)',
        'thor' : 'Horizontal sidelength of the rough bounding box (0 - 320 pixels)',
        'tvert' : 'Vertical sidelength of the rough bounding box (0 - 320 pixels)',
        'getpipe' : 'True active pipeline index of the camera (0 .. 9)',
        'json' : 'Full JSON dump of targeting results',
        'tclass' : 'Class ID of primary neural detector result or neural classifier result',
        'tc' : 'Get the average HSV color underneath the crosshair region as a NumberArray',
        # AprilTag
        'tid' : 'ID of the primary in-view AprilTag',
        # Camera Controls
        'ledMode' : "Sets limelight's LED state",
        'camMode' : "Sets limelight's operation mode",
        'pipeline' : "Sets limelight's current pipeline",
        'stream' : "Sets limelight's streaming mode",
        'snapshot' : 'Allows users to take snapshots during a match',
    }
    _ll_attributes_vector = {
        # AprilTag and 3D Data
        'botpose' : 'Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)',
        'botpose_wpiblue' : 'Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)',
        'botpose_wpired' : 'Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)',
        'camerapose_targetspace' : '3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))',
        'targetpose_cameraspace' : '3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))',
        'targetpose_robotspace' : '3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))',
        'botpose_targetspace' : '3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))',
        'camerapose_robotspace' : '3D transform of the camera in the coordinate system of the robot (array (6))',
        # Camera Control
        'crop' : '(Array) Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.',
        'camerapose_robotspace_set' : "(Array) Set the camera's pose in the coordinate system of the robot.",
    }
    _private_attributes = {
        'name' : 'The hostname for this limelight.',
        'nt' : 'NetworkTables table object for this limelight.',
    }

    @staticmethod
    def _identify_attribute_table(name):
        for attribute_table in (LimeLight._ll_attributes,
                                LimeLight._ll_attributes_vector,
                                LimeLight._private_attributes,
                                ):
            if name in attribute_table:
                return attribute_table
        # Fall through; didn't find.
        raise AttributeError(f"LimeLight doesn't know of attribute '{name}'.")

    def __init__(self, name="limelight"):
        self.name=name
        self.nt=NetworkTables.getTable(name)
    
    def getNumber(self, varname, default=0):
        return self.nt.getNumber(varname, default)

    def putNumber(self, varname, value):
        return self.nt.putNumber(varname, value)

    def getNumberArray(self, varname, default=()):
        return self.nt.getNumberArray(varname, default)

    def putNumberArray(self, varname, value):
        return self.nt.putNumberArray(varname, value)

    def __getattr__(self, name):
        attribute_table=self._identify_attribute_table(name)
        if(attribute_table is self._ll_attributes):
            return self.getNumber(name)
        elif(attribute_table is self._ll_attributes_vector):
            return self.getNumberArray(name)
        value = self.__dict__[name]
        return value

    def __setattr__(self, name, value):
        attribute_table=self._identify_attribute_table(name)
        if(attribute_table is self._ll_attributes):
            return self.putNumber(name, value)
        elif(attribute_table is self._ll_attributes_vector):
            return self.putNumberArray(name, value)
        self.__dict__[name]=value
        return value
