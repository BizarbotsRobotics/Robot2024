import ntcore
from wpilib import DriverStation, Timer
import wpilib.drive as driveLib
from Subsystems.Drive.SwerveModule import SwerveModule
from Util.MotorController import MotorControllerType, MotorType
from wpimath import geometry, kinematics, estimator, controller
from Subsystems.Vision.Vision import Vision
from constants import AutoConstants, SwerveConstants
import math
from phoenix5.sensors import WPI_Pigeon2
from commands2 import Subsystem
from pathplannerlib import auto, config, controller

class SwerveDrive(Subsystem):
    def __init__(self, vision: Vision):
        super().__init__()
        self.currentHeading = geometry.Rotation2d().degrees().real
        self.vision = vision
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        self.sd = inst.getTable("SmartDashboard")

        # Create Swerve Modules from Constants file
        self.initializeModules()

        #Initialize IMU 
        self.initializeIMU()

        # Create PID controller for heading correction
        self.headingPID = controller.PIDController(SwerveConstants.HEADING_P, SwerveConstants.HEADING_I, SwerveConstants.HEADING_D)
        self.headingPID.enableContinuousInput(0, 360)
        self.headingPID.setTolerance(1,1)
        # Sets the max speed according to our constants file
        self.setMaxSpeed()

        # Counter that will signal when to update our encoders with the absolute encoders
        self.moduleSynchronizationCounter = 0
        
        self.IMUOffset = geometry.Rotation3d()
        self.lastHeadingRadians = 0
        
        # Retrieve SwerveDrive Kinematic Object
        self.kinematics = self.getSwerveDriveKinematics()
        self.swerveDrivePoseEstimator = estimator.SwerveDrive4PoseEstimator(self.kinematics,
            self.yaw(),
            self.getModulePositions(),
            geometry.Pose2d(),
            (0.4,0,0.0),
            (0.4, 0.0, 0.1)
            #TODO Optimize these standard deviations later
           )
        auto.AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotVelocity, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.setChassisSpeeds, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            config.HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                controller.PIDConstants(.4, 0.0, 0.0), # Translation PID constants
                controller.PIDConstants(.7, 0.0, .1), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                config.ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.flipAlliance, # Supplier to control path flipping based on alliance color
            self# Reference to this subsystem to set requirements
        )

        
    def flipAlliance(self):
        alliance = DriverStation.getAlliance()
        print(alliance)
        return False
    
    def periodic(self):
        self.telemetry()
        #self.addVisionMeasurement()
        self.updateOdometry()

    def initializeModules(self):
        try:
            self.frontLeft = SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE, SwerveConstants.FRONT_LEFT_SWERVE,
                                        SwerveConstants.FRONT_LEFT_ENCODER_PORT, SwerveConstants.FRONT_LEFT_ENCODER_OFFSET, 
                                        MotorControllerType.SPARK_MAX, True)
            self.frontRight = SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE, SwerveConstants.FRONT_RIGHT_SWERVE,
                                        SwerveConstants.FRONT_RIGHT_ENCODER_PORT, SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET, 
                                        MotorControllerType.SPARK_MAX, True)
            self.backLeft = SwerveModule(SwerveConstants.BACK_LEFT_DRIVE, SwerveConstants.BACK_LEFT_SWERVE,
                                        SwerveConstants.BACK_LEFT_ENCODER_PORT, SwerveConstants.BACK_LEFT_ENCODER_OFFSET, 
                                        MotorControllerType.SPARK_MAX, True)
            self.backRight = SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE, SwerveConstants.BACK_RIGHT_SWERVE,
                                        SwerveConstants.BACK_RIGHT_ENCODER_PORT, SwerveConstants.BACK_RIGHT_ENCODER_OFFSET, 
                                        MotorControllerType.SPARK_MAX, True)
            self.swerveModules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]

        except Exception as e:
            raise Exception("Check ports in constants file or check for Incorrect can IDs")

    def initializeIMU(self):
        self.imu = WPI_Pigeon2(SwerveConstants.PIGEON_PORT)
        self.imu.reset()
        

    def getIMURawRotational3d(self):
        wxyz = self.imu.get6dQuaternion()[1]
        return geometry.Rotation3d(geometry.Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]))
    
    def getIMURotational3d(self):
        return self.getIMURawRotational3d().rotateBy(self.IMUOffset)
  
    def driveFieldOriented(self, velocity, centerOfRotationMeters=None):
        fieldOrientedVelocity = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(velocity, self.yaw())
        if centerOfRotationMeters is None:
            self.drive(fieldOrientedVelocity)
        else:
            self.drive(fieldOrientedVelocity, centerOfRotationMeters)

    def drivePathPlanner(self,velocity):
        self.driveF(velocity, False, geometry.Translation2d())
    
    def drive(self, velocity, centerOfRotationMeters):
        self.drive(velocity, False, centerOfRotationMeters)

    # def drive(self, translation, rotation, fieldRelative, isOpenLoop, centerOfRotationMeters):
    #     if fieldRelative:
    #         velocity = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.yaw())
    #     else:
    #         velocity = kinematics.ChassisSpeeds(translation.x, translation.y, rotation)

    #     self.drive(velocity, isOpenLoop, centerOfRotationMeters)

    def driveFR(self, translationX, translationY, rotation, fieldRelative, isOpenLoop):
        if translationX < .05 and translationX > -.05:
            translationX = 0

        if translationY < .05 and translationY > -.05:
            translationY = 0

        if rotation < .05 and rotation > -.05:
            rotation = 0
        

        translationX = translationX * 4.5
        translationY = translationY * 4.5
        rotation = rotation * 4.5

        
        velocity = kinematics.ChassisSpeeds(translationX, translationY, rotation)

        self.driveF(velocity, True, geometry.Translation2d(0,0))
    
    def limelight_aim_proportional(self):
        kP = .01

        targetingAngularVelocity = self.vision.getSpeakerCoords()[0] * kP
        targetingAngularVelocity *= 1
        targetingAngularVelocity *= -1.0

        return targetingAngularVelocity

    def limelight_range_proportional(self):    
        kP = .01
        targetingForwardSpeed = self.vision.getSpeakerCoords()[1] * kP
        targetingForwardSpeed *= 1
        targetingForwardSpeed *= -1.0
        return targetingForwardSpeed

    def driveF(self, velocity, isOpenLoop, centerOfRotationMeters):
        
        if SwerveConstants.VELOCITY_CORRECTION:
            dtConstant = 0.009
            robotPoseVel = geometry.Pose2d(velocity.vx * dtConstant,
                                            velocity.vy * dtConstant,
                                            geometry.Rotation2d(velocity.omega * dtConstant))
            twistVel = self.poseLock(robotPoseVel)

            velocity = kinematics.ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
                                        twistVel.dtheta / dtConstant)
        
        if SwerveConstants.HEADING_CORRECTION:
            if abs(velocity.omega) < 0.01:
                velocity.omega = self.headingPID.calculate(self.lastHeadingRadians, self.yaw().radians())
            else:
                self.lastHeadingRadians = self.yaw()
        swerveModuleStates = self.kinematics.toSwerveModuleStates(velocity, centerOfRotationMeters)

        self.setRawModuleStates(swerveModuleStates, isOpenLoop)
    
    def setRawModuleStates(self, desiredStates, isOpenLoop):
        if SwerveConstants.ATTAINABLE_MAX_TRANSLATIONAL_SPEED_METERS_PER_SECOND != 0 or SwerveConstants.ATTAINABLE_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND != 0:
            kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, self.getRobotVelocity,
                                                  SwerveConstants.MAX_SPEED,
                                                  SwerveConstants.ATTAINABLE_MAX_TRANSLATIONAL_SPEED_METERS_PER_SECOND,
                                                  SwerveConstants.ATTAINABLE_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND)
        counter = 0
        for swerve in self.swerveModules:
            swerve.setDesiredState(desiredStates[counter], isOpenLoop)
            counter+=1

    def setModuleStates(self, desiredStates, isOpenLoop):
        self.setRawModuleStates(self.kinematics.toSwerveModuleStates(self.kinematics.toChassisSpeeds(desiredStates)),isOpenLoop)
    
    def setChassisSpeeds(self, chassisSpeeds):
        self.setRawModuleStates(self.kinematics.toSwerveModuleStates(chassisSpeeds), False)

    def getPose(self):
        poseEstimation = self.swerveDrivePoseEstimator.getEstimatedPosition()
        return poseEstimation

    def resetOdometry(self, pose):
        self.swerveDrivePoseEstimator.resetPosition(self.yaw(), self.getModulePositions(), pose)
        self.kinematics.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, pose.rotation()))


    def getFieldVelocity(self):
        return kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(self.kinematics.toChassisSpeeds(self.getStates()), self.yaw().unaryMinus())

    def getRobotVelocity(self):
        print(self.kinematics.toChassisSpeeds(self.getStates()).vx)
        return self.kinematics.toChassisSpeeds(self.getStates())
    
    def getStates(self):
        states = []
        counter = 0
        for swerve in self.swerveModules:
            states.insert(counter, swerve.getState())
            counter+=1
        return states
    
    def poseLock(self, transform):
        kEps          = 1E-9
        dtheta        = transform.rotation().radians()
        half_dtheta   = 0.5 * dtheta
        cos_minus_one = transform.rotation().cos() - 1.0
        if abs(cos_minus_one) < kEps:
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta
        else:
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.rotation().sin()) / cos_minus_one
        translation_part = transform.translation().rotateBy(geometry.Rotation2d(halftheta_by_tan_of_halfdtheta,-half_dtheta))
        return geometry.Twist2d(translation_part.x, translation_part.y, dtheta)
    
    # Sets gyro to 0
    def zeroGyro(self):
        if self.imu != None:
            self.setGyroOffset(self.getIMURawRotational3d())
        self.lastHeadingRadians = 0
        self.resetOdometry(geometry.Pose2d(self.getPose().translation(), geometry.Rotation2d()))
    
    # Basically a X-axis
    def yaw(self):
        if self.imu != None:
            return geometry.Rotation2d(self.__get_gyro_heading(self.getIMURotational3d().Z().real))
        else:
            return geometry.Rotation2d()

    def __get_gyro_heading(self, angle) -> float:
        angle = math.degrees(angle)
        angle = math.fmod(-angle, 360)

        if angle < 0:
            return math.radians(angle if angle >= -180 else angle + 360)
        else:
            return math.radians(angle if angle <= 180 else angle - 360)
    # Z-axis
    def getPitch(self):
        if self.imu != None:
            return geometry.Rotation2d(self.getIMURotational3d().y)
        else:
            return geometry.Rotation2d()
    # Y-axis
    def getRoll(self):
        if self.imu != None:
            return geometry.Rotation2d(self.getIMURotational3d().x)
        else:
            return geometry.Rotation2d()
    
    def getGyroRotation3d(self):
        if self.imu != None:
            return self.getIMURotational3d()
        else:
            return geometry.Rotation2d()
        
    def getAcceleration(self):
        if self.imu != None:
            acceleration = self.imu.getBiasedAccelerometer()
            return geometry.Translation3d(acceleration[0], acceleration[1], acceleration[2])
        else:
            return geometry.Translation3d()
        
    def setMotorIdleMode(self, brake):
        """
        Sets the motor's brake mode

        Args:
            brake (bool): True for brake, False for idle
        """
        for swerve in self.swerveModules:
            swerve.setMotorBrake(brake)
            
    def setMaxSpeed(self):
        """_summary_
        """
        self.maxSpeed = SwerveConstants.MAX_SPEED

    def getSwerveModulePoses(self, robotPose):
        poses = []
        poses.append(robotPose.transformBy(geometry.Transform2d(geometry.Translation2d(SwerveConstants.FRONT_LEFT_CORDS.x, SwerveConstants.FRONT_LEFT_CORDS.y), 
                                                                self.swerveModules[0].getState().angle)))
        poses.append(robotPose.transformBy(geometry.Transform2d(geometry.Translation2d(SwerveConstants.FRONT_RIGHT_CORDS.x, SwerveConstants.FRONT_RIGHT_CORDS.y), 
                                                                self.swerveModules[1].getState().angle)))
        poses.append(robotPose.transformBy(geometry.Transform2d(geometry.Translation2d(SwerveConstants.BACK_LEFT_CORDS.x, SwerveConstants.BACK_LEFT_CORDS.y), 
                                                                self.swerveModules[2].getState().angle)))
        poses.append(robotPose.transformBy(geometry.Transform2d(geometry.Translation2d(SwerveConstants.BACK_RIGHT_CORDS.x, SwerveConstants.BACK_RIGHT_CORDS.y), 
                                                                self.swerveModules[3].getState().angle)))
        return poses
    
    def updateOdometry(self):
        try:
            self.swerveDrivePoseEstimator.update(self.yaw(), self.getModulePositions())
            self.currentHeading = self.swerveDrivePoseEstimator.getEstimatedPosition().rotation().degrees().real
            sumVelocity = 0.0
            for swerve in self.swerveModules:
                moduleState = swerve.getState()
                sumVelocity += abs(moduleState.speed)
            
            # If the velocity of the robot is less than 1% we reseed the encoders
            self.moduleSynchronizationCounter+=1
            if sumVelocity <= .01 and self.moduleSynchronizationCounter > 5:
                self.synchronizeModuleEncoders()
                self.moduleSynchronizationCounter = 0
        except Exception as e:
            raise e
        return None

    def synchronizeModuleEncoders(self):
        """
        Loops through each swerve module and updates the relative encoder with the absolute encoders position.
        """
        for swerve in self.swerveModules:
            swerve.queueSynchronizeEncoders()


    def lockPose(self):
        """
        Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep 
        the current pose.
        """
        counter = 1
        for swerve in self.swerveModules:
            if counter%2:
                angle = 45
            else:
                angle = 135
            desiredState = kinematics.SwerveModuleState(0, angle)
            swerve.setDesiredState(desiredState)
            self.kinematics.toSwerveModuleStates(kinematics.ChassisSpeeds())
            counter+=1

    def getSwerveDriveKinematics(self) -> kinematics.SwerveDrive4Kinematics:
        """
        Return the Kinematics object for the current swerve drive using the values in our constants file.

        Raises:
            Exception: Constants are not set or not set correctly.

        Returns:
            kinematics.SwerveDrive4Kinematics : Swerve Drive Kinematics Object
        """
        try:
            m_frontLeftLocation = geometry.Translation2d(SwerveConstants.FRONT_LEFT_CORDS['x'], SwerveConstants.BACK_LEFT_CORDS['y'])
            m_frontRightLocation = geometry.Translation2d(SwerveConstants.FRONT_RIGHT_CORDS['x'], SwerveConstants.FRONT_RIGHT_CORDS['y'])
            m_backLeftLocation = geometry.Translation2d(SwerveConstants.BACK_LEFT_CORDS['x'], SwerveConstants.BACK_LEFT_CORDS['y'])
            m_backRightLocation = geometry.Translation2d(SwerveConstants.BACK_RIGHT_CORDS['x'], SwerveConstants.BACK_RIGHT_CORDS['y'])
            return kinematics.SwerveDrive4Kinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation)
        except Exception as e:
            raise Exception("Check your constants folder")
    

    def getModulePositions(self, invertOdometry=True) -> tuple:
        """
        Returns the module position as a tuple.

        Args:
            invertOdometry (bool, optional): Inverts the odometry. Defaults to False.

        Returns:
            tuple: Position of the module.
        """
        counter = 0
        positions = []
        for swerve in self.swerveModules:
            positions.insert(counter, swerve.getSwerveModulePosition())
            if invertOdometry:
                positions[counter].distance *= -1
            counter+=1
        return tuple(positions)

    def addVisionMeasurement(self):
        """
        Adds vision measurements to make our odometry more accurate.

        Args:
            robotPose (_type_): _description_
            timestamp (_type_): _description_
        """
        visionPose = self.vision.getRobotPose()
        if visionPose[0] != 0:
            newPose = geometry.Pose2d(geometry.Translation2d(visionPose[0], visionPose[1]), geometry.Rotation2d.fromDegrees(visionPose[5]))
            self.swerveDrivePoseEstimator.addVisionMeasurement(newPose, Timer.getFPGATimestamp())
            newOdometry = geometry.Pose2d(self.swerveDrivePoseEstimator.getEstimatedPosition().translation(),
                                        newPose.rotation()).translation()
            self.setGyroOffset(geometry.Rotation3d(0, 0, newPose.rotation().getRadians()))
            self.resetOdometry(newOdometry)

    def setGyroOffset(self, offset):
        """
        Sets an offest to the gyro.

        Args:
            offset (number): Offset to be set.
        """
        self.IMUOffset = offset
    
    def setGyro(self, gyro):
        """
        Sets the gyro offset.

        Args:
            gyro (_type_): _description_
        """
        self.setGyroOffset(self.getIMURawRotational3d().minus(gyro))

    def driveAimAtPoint(self, xRequest, yRequest, rotateRequest, targetPoint: geometry.Translation2d, reversed, velocityCorrection):
        pass

    def getDistanceFromSpeaker(self):
        distance = math.dist([.14, 5.54], [self.getPose().Y().real, self.getPose().X().real])
        return distance
    
    def getAngleFromSpeaker(self):
        return  math.degrees(math.atan2(5.54 - self.getPose().X().real, .14 - self.getPose().Y().real))
    
    def getDistanceFromAmp(self):
        distance = math.dist([8.20, 1.78], [self.getPose().X().real, self.getPose().Y().real])
        return distance
    
    def getAngleFromAmp(self):
        return math.atan2(1.78 - self.getPose().Y().real, 8.20 - self.getPose().X().real)

    def headingCalculate(self, targetHeadingAngleRadians):
        self.sd.putNumber("current heading", self.currentHeading)
        self.sd.putNumber("target heading", targetHeadingAngleRadians)
        return -self.headingPID.calculate(self.getPose().rotation().rotateBy(geometry.Rotation2d(math.pi)).degrees().real, targetHeadingAngleRadians)

    def telemetry(self):
        """
        Enables Debug in Driver Station
        """
        for swerve in self.swerveModules:
            swerve.debug()
        
        # self.sd.putNumber("Pose X", self.getPose().X().real)
        # self.sd.putNumber("Pose Y", self.getPose().Y().real)
        # self.sd.putNumber("Pose Angle", self.getPose().rotation().degrees().real)
        # self.sd.putNumber("Speaker Distance", self.getDistanceFromSpeaker())
        # self.sd.putNumber("Speaker Angle", self.getAngleFromSpeaker())

        
