package frc.robot;

public final class Constants {
    public static final class MotorConstants{
        public static final int kFrontRightMotor = 1;
        public static final int kFrontLeftMotor = 2;
        public static final int kAbsoluteEncoderPort = 0;

        public static final double kRotationP = 0.0050;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        public static final double kRunningSpeed = 0.03;
        public static final double kHomePosition = 1.22e-1;
    }
    public static final class JoystickPortConstants {
        public static final int kOperatorJoystick = 1;
        public static final int kDriverJoystick = 0;
    }
    public static final class ButtonBindingConstants {
        public static final int kSpoolMotorButton = 1;
        public static final int kStopMotorButton = 2;
    }
    
}
