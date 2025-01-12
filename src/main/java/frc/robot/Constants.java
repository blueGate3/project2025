package frc.robot;

public final class Constants {
    public static final class IntakeConstants { 
        // Roller motors:
        public static final int kRightUpperMotor = 0;
        public static final int kRightLowerLeftMotor = 0;

        public static final int kLeftUpperMotor = 0;
        public static final int kLeftLowerLeftMotor = 0;
        
    }
    public static final class SwerveConstants { 
        // Module 1:
        public static final int kFrontRightDriveMotor = 0;
        public static final int kFrontRightTurnMotor = 0;
        public static final int kFrontRightEncoderPort = 0;
        // Module 2:
        public static final int kFrontLeftDriveMotor = 0;
        public static final int kFrontLeftTurnMotor = 0;
        public static final int kFrontLeftEncoderPort = 0;
        // Module 3:
        public static final int kBackRightDriveMotor = 0;
        public static final int kBackRightTurnMotor = 0;
        public static final int kBackRightEncoderPort = 0;
        // Module 4
        public static final int kBackLeftDriveMotor = 0;
        public static final int kBackLeftTurnMotor = 0;
        public static final int kBackLeftEncoderPort = 0;

        public static final double kRotationP = 0.0; // NO PID CURRENTLY
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

    }
    public static final class IOConstants {
        public static final int kOperatorJoystick = 1;
        public static final int kDriverJoystick = 0;
    }
    public static final class BindingConstants {
        // No current buttons have been added
    }

    public static final class SparkMaxIDs {
        //swerve is already taken care of in SwerveConstants
    }
    
}
