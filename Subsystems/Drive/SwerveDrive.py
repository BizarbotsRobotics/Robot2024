from Subsystems.Drive.SwerveModule import SwerveModule
from Util.MotorController import MotorControllerType, MotorType
from wpimath import geometry, kinematics, estimator, controller
from constants import SwerveConstants
import math
from phoenix6.hardware import pigeon2
from commands2 import Subsystem

class SwerveDrive(Subsystem):
    def __init__(self):
        super().__init__()

        # Create Swerve Modules from Constants file
        self.initializeModules()

        #Initialize IMU 
        self.initializeIMU()

        # Create PID controller for heading correction
        self.headingPID = controller.PIDController(SwerveConstants.HEADING_P, SwerveConstants.HEADING_I, SwerveConstants.HEADING_D)

        # Sets the max speed according to our constants file
        self.setMaxSpeed()

        # Counter that will signal when to update our encoders with the absolute encoders
        self.moduleSynchronizationCounter = 0
        
        self.IMUOffset = 0
        self.lastHeadingRadians = 0
        
        # Retrieve SwerveDrive Kinematic Object
        self.kinematics = self.getSwerveDriveKinematics()
        self.swerveDrivePoseEstimator = estimator.SwerveDrive4PoseEstimator(self.kinematics,
            self.yaw(),
            self.getModulePositions(),
            geometry.Pose2d(geometry.Translation2d(0, 0), geometry.Rotation2d.fromDegrees(0)),
            (0.1,0.1,0.1),
            (0.9, 0.9, 0.9)
            #TODO Optimize these standard deviations later
           )
        #self.pushOffsetsToControllers()
        
    def periodic(self):
        pass

    def initializeModules(self):
        try:
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
            self.backRight.setDriveInverted()
            self.backLeft.setDriveInverted()
            self.swerveModules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]
        except Exception as e:
            raise Exception("Check ports in constants file or check for Incorrect can IDs")

    def initializeIMU(self):
        try:
            self.imu = pigeon2(SwerveConstants.PIGEON_PORT) 
            self.zeroGyro()
        except Exception as e:
            self.imu = None

    def getIMURawRotational3d(self):
        wxyz = []
        self.imu.get6dQuaternion(wxyz)
        return geometry.Rotation3d(geometry.Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]))
    
    def getIMURotational3d(self):
        return self.getRawRotation3d().rotateBy(self.IMUOffset)
  
    def driveFieldOriented(self, velocity, centerOfRotationMeters=None):
        fieldOrientedVelocity = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(velocity, self.yaw())
        if centerOfRotationMeters is None:
            self.drive(fieldOrientedVelocity)
        else:
            self.drive(fieldOrientedVelocity, centerOfRotationMeters)

    def drive(self,velocity):
        self.drive(velocity, False, geometry.Translation2d())
    
    def drive(self, velocity, centerOfRotationMeters):
        self.drive(velocity, False, centerOfRotationMeters)

    def drive(self, translation, rotation, fieldRelative, isOpenLoop, centerOfRotationMeters):
        if fieldRelative:
            velocity = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.yaw())
        else:
            velocity = kinematics.ChassisSpeeds(translation.x, translation.y, rotation)

        self.drive(velocity, isOpenLoop, centerOfRotationMeters)

    def drive(self, translation, rotation, fieldRelative, isOpenLoop):
        if fieldRelative:
            velocity = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.yaw())
        else:
            velocity = kinematics.ChassisSpeeds(translation.x, translation.y, rotation)

        self.drive(velocity, isOpenLoop, geometry.Translation2d())
    
    def drive(self, velocity, isOpenLoop, centerOfRotationMeters):
        
        # if SwerveConstants.VELOCITY_CORRECTION:
        #     dtConstant = 0.009
        #     robotPoseVel = geometry.Pose2d(velocity.vx * dtConstant,
        #                                     velocity.vy * dtConstant,
        #                                     geometry.Rotation2d(velocity.omega * dtConstant))
        #     twistVel = self.poseLock(robotPoseVel)

        #     velocity = kinematics.ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
        #                                 twistVel.dtheta / dtConstant)
        
        # if SwerveConstants.HEADING_CORRECTION:
        #     if abs(velocity.omega) < 0.01:
        #         velocity.omega = self.headingPID.calculate(self.lastHeadingRadians, self.yaw().radians())
        #     else:
        #         self.lastHeadingRadians = self.yaw()

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
        self.swerveDrivePoseEstimator.resetPosition(pose.rotation(), self.getModulePositions(), pose)
        self.kinematics.toSwerveModuleStates(kinematics.ChassisSpeeds.freomFieldRelativeSpeeds(0, 0, 0, pose.rotation()))


    def getFieldVelocity(self):
        return kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(self.kinematics.toChassisSpeeds(self.getStates()), self.yaw().unaryMinus())

    def getRobotVelocity(self):
        return self.kinematics.toChassisSpeeds(self.getStates())
    
    def getStates(self):
        states = []
        counter = 0
        for swerve in self.swerveModules:
            states.insert(counter, swerve.getState())
            counter+=1
        return states
    
    # Cannot Drive Robot, Wheels Are Locked
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
            return geometry.Rotation2d(self.getIMURotational3d().getZ())
        else:
            return geometry.Rotation2d()
        
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
        
    # Tells robot to not move
    def setMotorIdleMode(self, brake):
        for swerve in self.swerveModules:
            swerve.setMotorBrake(brake)
            
    def setMaxSpeed(self):
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
            sumVelocity = 0.0
            for swerve in self.swerveModules:
                moduleState = swerve.getState()
                sumVelocity += abs(moduleState.speed)
            
            # If the velocity of the robot is less than 1% we reseed the encoders
            self.moduleSynchronizationCounter+=1
            if sumVelocity <= .01 and self.moduleSynchronizationCounter > 5:
                #self.synchronizeModuleEncoders()
                self.moduleSynchronizationCounter = 0
        except Exception as e:
            raise e
        return None

    def synchronizeModuleEncoders(self):
        pass
        #for swerve in self.swerveModules:
            #swerve.queueSynchronizeEncoders()
        #TODO add this method in swerve module class


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

    def getSwerveDriveKinematics(self):
        try:
            m_frontLeftLocation = geometry.Translation2d(SwerveConstants.FRONT_LEFT_CORDS['x'], SwerveConstants.BACK_LEFT_CORDS['y'])
            m_frontRightLocation = geometry.Translation2d(SwerveConstants.FRONT_RIGHT_CORDS['x'], SwerveConstants.FRONT_RIGHT_CORDS['y'])
            m_backLeftLocation = geometry.Translation2d(SwerveConstants.BACK_LEFT_CORDS['x'], SwerveConstants.BACK_LEFT_CORDS['y'])
            m_backRightLocation = geometry.Translation2d(SwerveConstants.BACK_RIGHT_CORDS['x'], SwerveConstants.BACK_RIGHT_CORDS['y'])
            return kinematics.SwerveDrive4Kinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation)
        
        except Exception as e:
            raise Exception("Check your constants folder")
    

    def getModulePositions(self, invertOdometry=False):
        counter = 0
        positions = []
        for swerve in self.swerveModules:
            positions.insert(counter, swerve.getSwerveModulePosition())
            if invertOdometry:
                positions[counter].distanceMeters *= -1
            counter+=1
        return tuple(positions)
    
    def pushOffsetsToControllers(self):
        print("emily")
        for swerve in self.swerveModules:
            swerve.pushOffsetsToControllers()

    def addVisionMeasurement(self,robotPose, timestamp):
        self.swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp)
        newOdometry = geometry.Pose2d(self.swerveDrivePoseEstimator.getEstimatedPosition().translation(),
                                    robotPose.rotation()).translation()
        self.setGyroOffset(geometry.Rotation3d(0, 0, robotPose.rotation().getRadians()))
        self.resetOdometry(newOdometry)

    def setGyroOffset(self, offset):
      self.IMUOffset = offset
    
    def setGyro(self, gyro):
        self.setGyroOffset(self.getIMURawRotational3d().minus(gyro))

    def getTargetSpeeds(self, xInput, yInput, headingX, headingY):
        pass

    def enableTelemetry(self):
        for swerve in self.swerveModules:
            swerve.debug()
    

        
