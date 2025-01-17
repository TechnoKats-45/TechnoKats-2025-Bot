package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

//import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
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
        public static final String LIMELIGHT_FRONT_TABLE = "limelight-front";
        public static final String LIMELIGHT_LEFT_TABLE = "limelight-left";
        public static final String LIMELIGHT_RIGHT_TABLE = "limelight-right";

        public static Matrix<N3, N1> ODOM_STD_DEV;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
    }
}
