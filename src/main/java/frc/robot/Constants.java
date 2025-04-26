package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public class CradleConst {
        public static final int leftID = 0;
        public static final int rightID = 0;
    }

    public class ElevatorConst { //in the spirit of consistency, unfortunately we must convert everything into meters. 
        public static final double inchToMeter = 0.0254; //multiply inches by to get meters.
        public static final double spoolDiameter = 0.0254; //our diameter is 1 inch, but it seemed cleaner to have two variables im sorry if i offended anyone. 
        public static final double elevatorOffset = 0.11; //meters, it' disitance from bottom of tray to ground.
        public static final double elevatorGearRatio = 4; //9:1. may switch to 4,5 or 6.
        public static final double rotationToMeterScaler = (Math.PI*spoolDiameter)/elevatorGearRatio; //math should be right, just multiply to get meters from rotations
        public static final int maxCurrent = 60;

        public static final int leftID = 11;
        public static final int rightID = 10;

        //elevator starts 5.5 inches, or .14 meters, off the ground, form bottom tip of tray to floor
        public static final double homePosition = elevatorOffset; 
        public static final double L2Height = 0.8763;
        public static final double L3Height = 1.22555;
        public static final double L4Height = 1.8415;

        //im sorry for capitalizing L at the beginning but it looked weird otherwise
        public static final TrapezoidProfile.State homeState = new TrapezoidProfile.State(homePosition, 0);
        public static final TrapezoidProfile.State L2state = new TrapezoidProfile.State(L2Height, 0); //we score L1 by shooting at L2 and missing
        public static final TrapezoidProfile.State L3state = new TrapezoidProfile.State(L3Height, 0);
        public static final TrapezoidProfile.State L4state = new TrapezoidProfile.State(L4Height, 0);

        /*from https://www.reca.lc/linear
         * 9:1 w/ 80% efficiency, 53lb load, 2 NEO Vortex, 1.708 m distance, 60A max current: 
         * .95 max vel, 17.35 max accel, .44V kG, 11.97 kV, .07 kA, 1.82 seconds total from home to L4
         * 6:1 w/ 80% efficiency, 53lb load, 2 NEO Vortex, 1.708 m distance, 60A max current: 
         * 1.38 max vel, 14.52 max accel, .67V kG, 11.97 kV, .11 kA, 1.38 seconds total from home to L4
         */

         //right now at 9:1, 245.2 is stall load, 1.82 seconds 0-L4
        public static final double kMaxVelocity = 2.09; //m/s
        public static final double kMaxAcceleration = 17.75; //m/s^2
        public static final double kDt = 0.02; //delta time im assuming, things are called every 20 ms so should be good
        public static final double kP = 3.8; //raise?
        public static final double kI = 0.0;
        public static final double kD = 0.7;
        public static final double kS = 1.1; //todo 
        public static final double kG = .44; //real, V
        public static final double kV = 2.66; //real, V*s/m. 
        public static final double kA = .07; //real, V*s^2/m 
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

        public static final double speedLimiter = .62;
    }

    public class AlignConst {
        public static final String reefLimelight = ""; //change to "reef" when we get second limnelight on friday
        public static final String hpLimelight = "hp";

        public static final double pX = 0.7;
        public static final double pY = 0.1;
        public static final double pRot = 0.003;

        public static final double xTol = .02; //meters
        public static final double yTol = .05; //meters
        public static final double rotTol = .5; //degrees
        
        public static final double xSet = .165;
        public static final double ySet = .1;

        
        public static final double hpX = 0.0;
    }

    	// Auto constants
	public static final double X_REEF_ALIGNMENT_P = 0.65; //.8
	public static final double Y_REEF_ALIGNMENT_P = 0.18; //.4
	public static final double ROT_REEF_ALIGNMENT_P = 0.02;

    //Distance between poles is 13 inches from center to center
    //Tolerances are in meters. 
	public static final double X_SETPOINT_REEF_ALIGNMENT = 0.165;  //distance from center of pole to center of limelight
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.015;

	public static final double Y_SETPOINT_REEF_ALIGNMENT = .83;  //Distance from limelight to Apriltag with bumpers (along y axis) //.6858
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = .05;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = -10;  // Rotation, in degrees
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = .05;

    public static final double LIMELIGHT_HEIGHT = .381; //height of limelight from ground in meters
    public static final double APRILTAG_HEIGHT = .305; //Center of apriltag to ground in meters
    public static final double LIMELIGHT_OFFSET = LIMELIGHT_HEIGHT-APRILTAG_HEIGHT;

}