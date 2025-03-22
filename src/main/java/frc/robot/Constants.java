package frc.robot;

public class Constants {
    public class CradleConst {
        public static final int leftID = 0;
        public static final int rightID = 0;
    }

    public class ElevatorConst {
        public static final int leftID = 0;
        public static final int rightID = 0;
    }
    public class DriveConst {

        //CAN IDs for all our drivesystem
        public static final int FLDrive = 1;
        public static final int FLTURN = 2;
        public static final int FRDrive = 3;
        public static final int FRTurn = 4;
        public static final int BRDrive = 5;
        public static final int BRTurn = 6;
        public static final int BLDrive = 7;
        public static final int BLTurn = 8;

        /* something useful later
        * - m/s to rpm formula: RPM = ((Velocity in m/s)/(circumference)) *60 (you multiply by 60 to convert revolutions per second to revolutions per minute)
        * - with gear ratio: rpm of output = rpm of motor * (gear ratio/output gear teeth)
        */
        public static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4")
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;
        public static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
        public static final double drivingWheelGearRatio = 5.36; //L3 gear ratio for driving, max velocity of 19.3 ft/sec
        public static final double rpmToVelocityScaler = (kWheelCircumference/(drivingWheelGearRatio*60)); //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        public static final double rotationsToDistanceScaler = kWheelCircumference / drivingWheelGearRatio; //x is number of rotations. 
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
        public static final double kMaxSpeed = 5.88; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
        public static final double turnEncoderScalar = 2* Math.PI;
    }
}