from MotorController import MotorControllerType, MotorType
from Subsystems import subsystem
from wpimath import geometry, kinematics
from Subsystems.Drive.SwerveModule import SwerveModule
from constants import SwerveConstants
from ctre.sensors import Pigeon2, WPI_Pigeon2


class SwerveDrive(subsystem):
    def __init__(self):

        # Create Swerve Modules from Constants file
        self.initializeModules()

        #Initialize IMU 
        self.initializeIMU()

        self.setMaxSpeed()

        # Retrieve SwerveDrive Kinematic Object
        self.kinematics = self.getSwerveDriveKinematics();

        pass

    def initializeModules(self):
        self.frontLeft = SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE, SwerveConstants.FRONT_LEFT_SWERVE,
                                       SwerveConstants.FRONT_LEFT_ENCODER_PORT, SwerveConstants.FRONT_LEFT_ENCODER_OFFSET, 
                                       MotorControllerType.SPARK_MAX)
        self.frontRight = SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE, SwerveConstants.FRONT_RIGHT_SWERVE,
                                       SwerveConstants.FRONT_RIGHT_ENCODER_PORT, SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET, 
                                       MotorControllerType.SPARK_MAX)
        self.backLeft = SwerveModule(SwerveConstants.BACK_LEFT_DRIVE, SwerveConstants.BACK_LEFT_SWERVE,
                                       SwerveConstants.BACK_LEFT_ENCODER_PORT, SwerveConstants.BACK_LEFT_ENCODER_OFFSET, 
                                       MotorControllerType.SPARK_MAX)
        self.backRight = SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE, SwerveConstants.BACK_RIGHT_SWERVE,
                                       SwerveConstants.BACK_RIGHT_ENCODER_PORT, SwerveConstants.BACK_RIGHT_ENCODER_OFFSET, 
                                       MotorControllerType.SPARK_MAX)
        self.swerveModules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]

    def initializeIMU(self):
        try:
            self.imu = WPI_Pigeon2(SwerveConstants.PIGEON_PORT) 
            self.zeroGyro()
        except Exception as e:
            self.imu = None

    def zeroGyro(self):
        if self.imu != None:
            self.imu.setOffset(self.imu.getRawRotation3d())
            #TODO reset odometry
    
    def getYaw(self):
        if self.imu != None:
            return geometry.Rotation2d.fromRadians(self.imu.getRotation3d().getZ())
        else:
            return None

    def getPitch(self):
        if self.imu != None:
            return geometry.Rotation2d.fromRadians(self.imu.getRotation3d().getY())
        else:
            return None

    def getRoll(self):
        if self.imu != None:
            return geometry.Rotation2d.fromRadians(self.imu.getRotation3d().getX())
        else:
            return None
    
    def getGyroRotation3d(self):
        if self.imu != None:
            return self.imu.getRotation3d()
        else:
            return None
        
    def getAcceleration(self):
        if self.imu != None:
            acceleration = self.imu.getBiasedAccelerometer()
            return geometry.Translation3d(acceleration[0], acceleration[1], acceleration[2])
        else:
            return None
        
    def setMotorIdleMode(self, brake):
        for swerve in self.swerveModules:
            swerve.setMotorBrake(brake)

    def setMaxSpeed(self):
        self.maxSpeed = SwerveConstants.MAX_SPEED

    def lockPose(self):
        """
        Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep 
        the current pose.
        """
        #TODO Implement
        pass

    def getSwerveDriveKinematics(self):
        try:
            m_frontLeftLocation = geometry.Translation2d(SwerveConstants.FRONT_LEFT_CORDS.x, SwerveConstants.BACK_LEFT_CORDS.y)
            m_frontRightLocation = geometry.Translation2d(SwerveConstants.FRONT_RIGHT_CORDS.x, SwerveConstants.FRONT_RIGHT_CORDS.y)
            m_backLeftLocation = geometry.Translation2d(SwerveConstants.BACK_LEFT_CORDS.x, SwerveConstants.BACK_LEFT_CORDS.y);
            m_backRightLocation = geometry.Translation2d(SwerveConstants.BACK_RIGHT_CORDS.x, SwerveConstants.BACK_RIGHT_CORDS.y);
            return kinematics.SwerveDrive4Kinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation)
        except:
            raise Exception("Check ports in constants file or check for Incorrect can IDs")
        
