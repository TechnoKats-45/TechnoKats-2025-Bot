package frc.robot;

import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants 
{
    public static final double STICK_DEADBAND = 0.1;

    public static final class Vision
    {
        public static final String LIMELIGHT_FRONT_TABLE = "limelight-front";
        public static final String LIMELIGHT_LEFT_TABLE = "limelight-left";
        public static final String LIMELIGHT_RIGHT_TABLE = "limelight-right";

        public static Matrix<N3, N1> ODOM_STD_DEV;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
    }

    public static final class Destinations
    {
        // For Coral Scoring - Numbers taken from PathPlanner manually
        public static final Pose2d A = new Pose2d(3.146, 4.179, Rotation2d.fromDegrees(0));
        public static final Pose2d B = new Pose2d(3.146, 3.861, Rotation2d.fromDegrees(0));
        public static final Pose2d C = new Pose2d(3.695, 2.976, Rotation2d.fromDegrees(60));
        public static final Pose2d D = new Pose2d(3.955, 2.803, Rotation2d.fromDegrees(60));
        public static final Pose2d E = new Pose2d(5.013, 2.813, Rotation2d.fromDegrees(120));
        public static final Pose2d F = new Pose2d(5.273, 2.947, Rotation2d.fromDegrees(120));
        public static final Pose2d G = new Pose2d(5.802, 3.861, Rotation2d.fromDegrees(180));
        public static final Pose2d H = new Pose2d(5.802, 4.179, Rotation2d.fromDegrees(180));
        public static final Pose2d I = new Pose2d(5.302, 5.083, Rotation2d.fromDegrees(240));
        public static final Pose2d J = new Pose2d(5.013, 5.247, Rotation2d.fromDegrees(240));
        public static final Pose2d K = new Pose2d(3.964, 5.257, Rotation2d.fromDegrees(300));
        public static final Pose2d L = new Pose2d(3.964, 5.103, Rotation2d.fromDegrees(300));

        // For Algae Cleaning - Numbers taken from PathPlanner manually
        public static final Pose2d AB = new Pose2d(3.146, 3.146, Rotation2d.fromDegrees(0));
        public static final Pose2d CD = new Pose2d(3.832, 3.832, Rotation2d.fromDegrees(60));
        public static final Pose2d EF = new Pose2d(5.134, 5.134, Rotation2d.fromDegrees(120));
        public static final Pose2d GH = new Pose2d(5.803, 4.017, Rotation2d.fromDegrees(180));
        public static final Pose2d IJ = new Pose2d(5.148, 5.171, Rotation2d.fromDegrees(240));
        public static final Pose2d KL = new Pose2d(3.823, 5.166, Rotation2d.fromDegrees(300));

        // For Coral Aquisition - Numbers taken from PathPlanner manually
        public static final Pose2d LeftCoral = new Pose2d(1.693, 7.354, Rotation2d.fromDegrees(130+180));   // TODO - get more accurate values from actual field - if we choose to use this
        public static final Pose2d RightCoral = new Pose2d(1.665, 0.696, Rotation2d.fromDegrees(220+180));  // TODO - get more accurate value from actual field - if we choose to use this

        // For Algae Scoring - Numbers taken from PathPlanner manually
        public static final Pose2d Barge = new Pose2d(7.755, 6.498, Rotation2d.fromDegrees(0.180));
        public static final Pose2d Processor = new Pose2d(5.965, 0.542, Rotation2d.fromDegrees(270));
    }
    
    public static final class Climber
    {
        public static final int climbAngle = 400;    // TODO - find actual angle
    }
    
    // Operator Button Board Mapping
    public static final class Button
    {
        public static final class height
        {
            public static final int L1 = 5;
            public static final int L2 = 4;
            public static final int L3 = 3;
            public static final int L4 = 2;
            public static final int Barge = 1;
            public static final int A1 = 16;
            public static final int A2 = 15;
        }

        public static final class location
        {
            public static final int A = 24;
            public static final int B = 13;
            public static final int C = 12;
            public static final int D = 11;
            public static final int E = 10;
            public static final int F = 9;
            public static final int G = 8;
            public static final int H = 19;
            public static final int I = 20;
            public static final int J = 21;
            public static final int K = 22;
            public static final int L = 23;
            
            public static final int LeftCoral = 17;
            public static final int RightCoral = 7;

            public static final int Barge = 18;
            public static final int Processor = 6;
        }

        public static final int H = 14;
    }

    public static final class Elevator
    {
        // There is a 12:1 reduction between the motor and the mechanism
    // and an additional 11.9:1 between the mechanism and the encoder.
    public static final double MOTOR_TO_MECHANISM = 12.0;       
    public static final double MECHANISM_TO_ENCODER = 11.9;       
    public static final double TOTAL_GEAR_REDUCTION = MOTOR_TO_MECHANISM * MECHANISM_TO_ENCODER;  // 142.8

    public static final int elevatorMotor1ID = 7;
    public static final int elevatorMotor2ID = 8;
    public static final int elevatorCANdiID = 9;

    public static final int elevatorAngleTolerance = 1;
    
        public static final class AnglePresets // TODO
        {
            public static final double L1 = 3.8;
            public static final double L2 = 5.9;  // 6 works    // was 5.9
            public static final double L3 = 7.8;  
            public static final double L4 = 10.3;   // was 10.3
            
            public static final double Barge = 10.4;
            public static final double A1 = 4.2;
            public static final double A2 = 6;
    
            public static final double handoffAngle = 3.35;  // GOOD!    // was 3.5

            public static final double Stow = 1.5;

            public static final double algaeCleanStartHeight = 5;
            public static final double algaeCleanEndHeight = 5;
        }
    }
    

    public static final class Carriage
    {
        public static final double algaeIntakeSpeed = -1;   // TODO - Tune
        public static final double algaeCleanTime = 2;      // TODO - Tune
        
        public static final double algaeScoreSpeed = 1;     // TODO - Tune
        public static final double algaeAngleTolerance = 1; // TODO - Tune

        ////////////////////////////////////////////////////////////////

        public static final double coralIntakeSpeed = 45;
        public static final double coralPassiveIntakeSpeed = -45;   // -45
        public static final double coralReverseSpeed = 45;
        public static final double coralSlowIntakeSpeed = -30;
        public static final double coralScoreSpeed = -25;   // was -35
        public static final double autoCoralScoreSpeed = -25;   // -15 for L4

        public static final class AnglePresets
        {
            public static final double A1 = 1;
            public static final double A2 = 2;
            public static final double algaeStowAngle = -.26;
            public static final double algaeScoreAngle = -.3;
            public static final double algaeCleanAngle = -.55;       // Was -0.45
            public static final double algaeFloorIntake = -.4;
        }
    }
}
