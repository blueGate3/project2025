package frc.robot;

public final class Constants {
    public static final class IntakeConstants { 
        public static final int kFrontRightMotor = 0;
        public static final int kFrontLeftMotor = 0;
        public static final int kAbsoluteEncoderPort = 0;

        public static final double kRotationP = 0.0; // NO PID CURRENTLY
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        public static final double kRunningSpeed = 0.0; //  None currently
        public static final double kHomePosition = 0.0; // No current home position
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
    
}
