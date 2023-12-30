class SwerveConstants:
    # Max Speed of drive modules
    MAX_SPEED = 12

    # PIGEON 2 IMU ID
    PIGEON_PORT = 2

    # Coordinates of swerve modules relative to the center of the robot
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
    FRONT_RIGHT_DRIVE = 1
    FRONT_LEFT_DRIVE = 2
    BACK_RIGHT_DRIVE = 3
    BACK_LEFT_DRIVE = 4

    # Swerve Motor IDs for Swerve Module
    FRONT_RIGHT_SWERVE = 5
    FRONT_LEFT_SWERVE = 6
    BACK_RIGHT_SWERVE = 7
    BACK_LEFT_SWERVE = 8

    # Swerve Encoder Offsets
    FRONT_RIGHT_ENCODER_OFFSET = .06
    FRONT_LEFT_ENCODER_OFFSET = .06
    BACK_RIGHT_ENCODER_OFFSET = .06
    BACK_LEFT_ENCODER_OFFSET = .06



