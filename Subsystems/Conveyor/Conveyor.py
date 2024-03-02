import commands2
import ntcore
import rev
import wpilib
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import ConveyorConstants
import playingwithfusion

class Conveyor(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        # Start smart dashboard
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()
        self.sd = self.inst.getTable("SmartDashboard")
        self.proxSensor = playingwithfusion.TimeOfFlight(1)


        self.conveyorMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ConveyorConstants.CONVEYOR_MOTOR)
        self.conveyorMotor.setEncoderPositionConversion(.33)
        self.conveyorMotor.setCurrentLimit(30)
        self.conveyorMotor.save()

        
    def periodic(self):
        self.setProxVal()
        self.telemetry()

    def telemetry(self):
        """
        Sends subsystem info to console or smart dashboard
        """
        self.sd.putNumber("Conveyor RPM", self.getConveyorRPM())
        self.sd.putBoolean("Note Stored", self.getNoteStored())
        self.sd.putNumber("Note Prox", self.prox)

    def getConveyorRPM(self):
        return self.conveyorMotor.getBuiltInEncoderPosition()
    
    def setConveyorPower(self, power):
        self.conveyorMotor.setPower(power)

    def setProxVal(self):
        self.prox = self.proxSensor.getRange()
        pass

    def getNoteStored(self):
        if self.prox < 100:
            return True
        return False





    

