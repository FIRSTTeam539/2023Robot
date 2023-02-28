package frc.robot;
// change
//crtl+c Crtl+V
public final class Constants {
        public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int kRightMotor1SparkMaxCANID = 1;
        public static final int kRightMotor2SparkMaxCANID = 2;
        public static final int kLeftMotor1SparkMaxCANID = 3;
        public static final int kLeftMotor2SparkMaxCANID = 4;

        public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRightEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftEncoderReversed = false; //was true
        public static final boolean kRightEncoderReversed = true; //was false

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = false;

        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;

        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    }
    public static final class ArmConstants {
        public static final int kShootMotorPort = 4;
        public static final int kArmSparkMaxCANID = 5;
        //Modifies speed of arm motor with the Left Stick
        public static final double karmRate = 0.40;
        //modifies shooting speed
        public static final double kshootRate = 1;
        // modifies intake speed
        public static final double kintakeRate = 1;
        //minium and maxium values of clamp (Max and min possitions of arm)
        public static final double kMin= -0.7;
        public static final double kMax= 0.7;
        //set value of deadzone
        public static final double kDeadzone = 0.2;
        // The offset of the arm from the horizontal in its neutral position,
        // measured from the horizontal
    
      }
    
      public static final class AutoConstants {
        public static final double kAutoTimeoutSeconds = 12;
        public static final double kAutoShootTimeSeconds = 7;
      }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
    }
    
}
