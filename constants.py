class SwerveConstants:
    # Max Speeds of drive modules
    MAX_SPEED = 12
    ATTAINABLE_MAX_TRANSLATIONAL_SPEED_METERS_PER_SECOND = 0
    ATTAINABLE_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = 0

    # Relative Encoder Conversion
    SWERVE_ENCODER_CONVERSION_FACTOR = (1.0 / ( 150.0 / 7.0 )) * 360
    DRIVE_ENCODER_CONVERSION_FACTOR = (1.0 / 6.75)

    # PIGEON 2 IMU ID
    PIGEON_PORT = 2

    # Coordinates of swer ve modules relative to the center of the robot
    FRONT_LEFT_CORDS = {'x':.381, 'y':.381}
    FRONT_RIGHT_CORDS = {'x':.381, 'y':-.381}
    BACK_LEFT_CORDS = {'x':-.381, 'y':.381}
    BACK_RIGHT_CORDS = {'x':-.381, 'y':-.381}

    # Absolute Encoder Ports for Swerve Module
    FRONT_RIGHT_ENCODER_PORT = 0
    FRONT_LEFT_ENCODER_PORT = 1
    BACK_RIGHT_ENCODER_PORT = 2
    BACK_LEFT_ENCODER_PORT = 3

    # Drive Motor IDs for Swerve Module
    FRONT_RIGHT_DRIVE = 4
    FRONT_LEFT_DRIVE = 2
    BACK_RIGHT_DRIVE = 6
    BACK_LEFT_DRIVE = 8

    # Swerve Motor IDs for Swerve Module
    FRONT_RIGHT_SWERVE = 3
    FRONT_LEFT_SWERVE = 1
    BACK_RIGHT_SWERVE = 5
    BACK_LEFT_SWERVE = 7

    # Swerve Encoder Offsets
    FRONT_RIGHT_ENCODER_OFFSET = .70
    FRONT_LEFT_ENCODER_OFFSET = .44
    BACK_RIGHT_ENCODER_OFFSET = .826
    BACK_LEFT_ENCODER_OFFSET = .553

    # Swerve Motor PID Constants
    SWERVE_P = .08
    SWERVE_I = 0
    SWERVE_D = 0
    SWERVE_FF = 0
    SWERVE_MIN_OUTPUT = -1
    SWERVE_MAX_OUTPUT = 1

    # Velocity Correction
    VELOCITY_CORRECTION = False
    HEADING_CORRECTION = False

    # Heading Correction PID
    HEADING_P = 0
    HEADING_I = 0
    HEADING_D = 0

class ConveyorConstants:
    # Conveyor Motor ID
    CONVEYOR_MOTOR = 9

class IntakeConstants:
    #Intake Motor ID
    PIVOT_MOTOR_1 = 1
    PIVOT_MOTOR_2 = 2
    INTAKE_MOTOR = 12

    # Sets the absolute encoder port
    INTAKE_ENCODER_PORT = 4
    #Intake Pivot Motor PID Constants
    INTAKEPIVOT_P = 25
    INTAKEPIVOT_I = 0
    INTAKEPIVOT_D = .5
    INTAKEPIVOT_FF = 0
    INTAKEPIVOT_MIN_OUTPUT = -1
    INTAKEPIVOT_MAX_OUTPUT = 1

class ShooterConstants:
    SHOOTER_BOTTOM_ID = 13
    SHOOTER_TOP_ID = 14
    SHOOTER_PIVOT_1_ID = 16
    SHOOTER_PIVOT_2_ID = 15
    INDEXER_1_ID = 17
    INDEXER_2_ID = 18

    #Shooter PID Constants
    SHOOTER_P = 0
    SHOOTER_I = 0
    SHOOTER_D = 0
    SHOOTER_FF = .00018
    SHOOTER_MIN_OUTPUT = -1
    SHOOTER_MAX_OUTPUT = 1

    SHOOTER_FULL_SPEED = 1
    SHOOTER_AMP_SPEED = 1
    SHOOTER_LAST_RESORT = 1
    SHOOTER_HALF_COURT = 1
    SHOOTER_ZERO = 0

    #Shooter Pivot PID Constants
    SHOOTERPIVOT_P = .012
    SHOOTERPIVOT_I = 0
    SHOOTERPIVOT_D = 0
    SHOOTERPIVOT_FF = 0
    SHOOTERPIVOT_MIN_OUTPUT = -1
    SHOOTERPIVOT_MAX_OUTPUT = 1
    
    #Indexer PID Constants
    INDEXER_P = 2
    INDEXER_I = 0
    INDEXER_D = 0
    INDEXER_FF = 0
    INDEXER_MIN_OUTPUT = -1
    INDEXER_MAX_OUTPUT = 1



    



