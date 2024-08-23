from math import tan
import commands2
import ntcore
import numpy
import rev
import wpilib
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import ConveyorConstants

class Vision(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        # Start smart dashboard
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()
        self.sd = self.inst.getTable("SmartDashboard")
        
    def periodic(self):
        self.telemetry()

    def telemetry(self):
        """
        Sends subsystem info to console or smart dashboard
        """
        self.sd.putBoolean("Note", self.noteDetected())
        self.sd.putNumber("Speaker X", self.getSpeakerCoords()[0])
        self.sd.putNumber("Speaker Y", self.getSpeakerCoords()[1])
        self.sd.putNumber("Spesker Distance", self.getSpeakerDistance())

    def noteDetected(self):
        return self.inst.getTable("limelight-note").getEntry("tclass").getString("") == "note"
    
    def getSpeakerCoords(self):
        return [self.inst.getTable("limelight").getEntry("tx").getDouble(0), self.inst.getTable("limelight").getEntry("ty").getDouble(0)]
    
    def seeTarget(self):
        return self.inst.getTable("limelight").getEntry("tv").getInteger(0) != 0
    
    def getAmpCoords(self):
        return [self.inst.getTable("limelight").getEntry("tx").getDouble(0), self.inst.getTable("limelight").getEntry("ty").getDouble(0)]
    
    def getAmpDistance(self):
        limelightMountAngleDegrees = 25.0
        limelightLensHeightInches = 12.0
        goalHeightInches = 48.0
        angleToGoalDegrees = limelightMountAngleDegrees + self.getAmpCoords()[1]
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)

        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)

        return distanceFromLimelightToGoalInches
    
    def getAmpDistance(self):
        limelightMountAngleDegrees = 25.0
        limelightLensHeightInches = 12.0
        goalHeightInches = 53
        angleToGoalDegrees = limelightMountAngleDegrees + self.getAmpCoords()[1]
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)

        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)

        return distanceFromLimelightToGoalInches
    
    def getSpeakerDistance(self):
        limelightMountAngleDegrees = 10.0
        limelightLensHeightInches = 15.0
        goalHeightInches = 57
        angleToGoalDegrees = limelightMountAngleDegrees + self.getAmpCoords()[1]
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)

        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)

        return distanceFromLimelightToGoalInches
       

    
    def getRobotPose(self):
        return self.inst.getTable("limelight").getEntry("botpose").getDoubleArray([0,0,0,0,0,0])
    
    # def changePipeline(self, number):
    #     self.inst.getTable("limelight").getEntry("pipeline").setInteger(number)