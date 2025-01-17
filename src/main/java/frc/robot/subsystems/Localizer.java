package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * The Localizer subsystem is responsible for maintaining the best
 * estimate of the robot's pose on the field by fusing:
 *   - Swerve odometry (wheel encoder + gyro angle)
 *   - Vision measurements (e.g., from Limelights / AprilTags)
 */
public class Localizer extends SubsystemBase 
{
    // Reference to the swerve drivetrain for retrieving odometry, states, etc.
    private final CommandSwerveDrivetrain swerve;

    // WPILib’s pose estimator: fuses sensor + vision to maintain robot pose
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * Constructs the Localizer with a reference to the swerve subsystem.
     * This uses your updated constructor approach.
     */
    public Localizer(CommandSwerveDrivetrain swerve) 
    {
        this.swerve = swerve;

        // Initialize pose estimator with kinematics, gyro angle, and initial pose
        // Adjust the getState() calls if needed to match your own naming/returns.
        // e.g., RawHeading should be a Rotation2d, ModulePositions an array of SwerveModulePosition, etc.
        poseEstimator = new SwerveDrivePoseEstimator
        (
            this.swerve.getKinematics(),
            this.swerve.getState().RawHeading,           // Rotation2d
            this.swerve.getState().ModulePositions,      // SwerveModulePosition[]
            this.swerve.getState().Pose,                 // Pose2d initial pose
            Constants.Vision.ODOM_STD_DEV,               // Standard deviations for state (e.g. [x, y, theta])
            Constants.Vision.VISION_STD_DEV_MULTITAG_FUNCTION.apply(1.0) 
            // or pass in your standard deviations for vision
        );
    }

    @Override
    public void periodic() 
    {
        // Update the pose estimator based on the current swerve heading & module states
        poseEstimator.update(
            this.swerve.getState().RawHeading,
            this.swerve.getState().ModulePositions
        );
    }

    /**
     * Adds a vision-based pose measurement to the pose estimator.
     * 
     * @param visionEstimatedPose Robot pose from vision
     * @param timestampSeconds    Timestamp at which measurement was valid
     * @param visionStdDevs       Standard deviations for vision measurement
     */
    public void addVisionMeasurement(Pose2d visionEstimatedPose, double timestampSeconds, double[] visionStdDevs) 
    {
        poseEstimator.addVisionMeasurement
        (
            visionEstimatedPose, 
            timestampSeconds, 
            Constants.Vision.VISION_STD_DEV_MULTITAG_FUNCTION.apply(1.0) 
        );
    }

    /**
     * Resets the pose to a known location/orientation.
     * 
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) 
    {
        poseEstimator.resetPosition(
            this.swerve.getState().RawHeading,
            this.swerve.getState().ModulePositions,
            pose
        );
    }

    /**
     * Returns the current best estimate of the robot’s pose on the field.
     */
    public Pose2d getCurrentPose() 
    {
        return poseEstimator.getEstimatedPosition();
    }
}
