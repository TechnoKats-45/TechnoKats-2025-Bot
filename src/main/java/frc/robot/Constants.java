package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

//import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

public class Constants 
{
    public static final class Vision
    {
        public static final String LIMELIGHT_FRONT_TABLE = "limelight-front";   // TODO
        public static final String LIMELIGHT_LEFT_TABLE = "limelight-left"; // TODO
        public static final String LIMELIGHT_RIGHT_TABLE = "limelight-right";   // TODO

        public static Matrix<N3, N1> ODOM_STD_DEV;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
    }

    public static final class Destinations
    {
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

        public static final Pose2d LeftCoral = new Pose2d(1.693, 7.354, Rotation2d.fromDegrees(130+180));   // TODO - get more accurate value
        public static final Pose2d RightCoral = new Pose2d(1.665, 0.696, Rotation2d.fromDegrees(220+180));  // TODO - get more accurate value

        public static final Pose2d Barge = new Pose2d(7.755, 6.498, Rotation2d.fromDegrees(0.180));
        public static final Pose2d Processor = new Pose2d(5.965, 0.542, Rotation2d.fromDegrees(270));
    }
    
    public static final class Climber
    {
        public static final double hopperReleaseAngle = 0;  // TODO
        public static final double climbAngle = 0;          // TODO
        public static final double floorAngle = 0;           // TODO
    }
    
    // Operator Button Board Mapping
    public static final class Button
    {
        public static final class height    // TODO
        {
            public static final int L1 = -1;
            public static final int L2 = -1;
            public static final int L3 = -1;
            public static final int L4 = -1;
            public static final int Barge = -1;
            public static final int A1 = -1;
            public static final int A2 = -1;
        }

        public static final class location  // TODO
        {
            public static final int A = -1;
            public static final int B = -1;
            public static final int C = -1;
            public static final int D = -1;
            public static final int E = -1;
            public static final int F = -1;
            public static final int G = -1;
            public static final int H = -1;
            public static final int I = -1;
            public static final int J = -1;
            public static final int K = -1;
            public static final int L = -1;
            
            public static final int LeftCoral = -1; 
            public static final int RightCoral = -1;

            public static final int Barge = -1;
            public static final int Processor = -1;
        }

        public static final int H = -1;
    }

    public static final class HeightPresets // TODO
    {
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;
        public static final double L4 = 0;
        public static final double Barge = 0;
        public static final double A1 = 0;
        public static final double A2 = 0;
    }
}
