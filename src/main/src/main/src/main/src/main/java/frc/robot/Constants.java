package frc.robot;

public final class Constants {
    public static final class ElevatorConstants { 
        // Roller motors:
        public static final int kSpoolMotorId = 0;
        
    }
    public static final class SwerveConstants { 
        // Module 1:
        public static final int kFrontRightDriveMotorId = 0;
        public static final int kFrontRightTurnMotorId = 0;
        public static final int kFrontRightEncoderPortId = 0;
        // Module 2:
        public static final int kFrontLeftDriveMotorId = 0;
        public static final int kFrontLeftTurnMotorId = 0;
        public static final int kFrontLeftEncoderPortId = 0;
        // Module 3:
        public static final int kBackRightDriveMotorId = 0;
        public static final int kBackRightTurnMotorIdId = 0;
        public static final int kBackRightEncoderPortId = 0;
        // Module 4
        public static final int kBackLeftDriveMotorId = 0;
        public static final int kBackLeftTurnMotorId = 0;
        public static final int kBackLeftEncoderPortId = 0;

        public static final double kPTurnVal = 0.0; // NO PID CURRENTLY
        public static final double kITurnVal = 0.0;
        public static final double kDTurnVal = 0.0;

    }
    public static final class IOConstants {
        public static final int kOperatorJoystick = 1;
        public static final int kDriverJoystick = 0;
    }
    public static final class BindingConstants {
        // No current buttons have been added
    }
    
}
