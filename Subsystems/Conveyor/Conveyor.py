import commands2
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import ConveyorConstants

class Conveyor(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.conveyorMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ConveyorConstants.CONVEYOR_MOTOR)
        
        

    def runConveyor(self, power):
        self.conveyorMotor.setPower(power)





    

