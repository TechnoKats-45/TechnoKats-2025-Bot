package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class Localizer extends SubsystemBase 
{

  private final CommandSwerveDrivetrain swerveDrivetrain;
  private final AprilTagFieldLayout fieldLayout;

  // WPILib’s recommended swerve pose estimator
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Vision vision; // Our Vision subsystem to get tag-based robot poses

  public Localizer(CommandSwerveDrivetrain swerveDrivetrain, Vision vision) 
  {
    this.swerveDrivetrain = swerveDrivetrain;
    this.vision = vision;

    // Load the official 2025 Reefscape field layout
    AprilTagFieldLayout tempLayout = null;
    tempLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    fieldLayout = tempLayout;

    // Construct the SwerveDrivePoseEstimator
    poseEstimator = new SwerveDrivePoseEstimator
    (
        swerveDrivetrain.getKinematics(),
        getGyroHeading(),                               // Rotation2d
        swerveDrivetrain.getState().ModulePositions,    // current module positions
        new Pose2d()                                    // start at origin by default
    );
  }

  /**
   * Get the current heading from the swerve subsystem's Pigeon (or other gyro).
   * Adjust if your CommandSwerveDrivetrain provides a method to get heading as Rotation2d.
   */
  private Rotation2d getGyroHeading() 
  {
    return swerveDrivetrain.getState().RawHeading;
  }

  @Override
  public void periodic() 
  {
    // 1) Update with our swerve module states, gyro heading, and current time
    double currentTime = Timer.getFPGATimestamp();
    poseEstimator.updateWithTime
    (
        currentTime,
        getGyroHeading(),
        swerveDrivetrain.getKinematics()
    );

    // 2) Get the multi-tag-based robot poses from each Limelight
    //    (These are the robot's pose as inferred from each detected tag.)
    List<Pose2d> frontPoses = vision.getFrontLimelightAllTagPoses2d();
    List<Pose2d> leftPoses  = vision.getLeftLimelightAllTagPoses2d();
    List<Pose2d> rightPoses = vision.getRightLimelightAllTagPoses2d();

    // 3) If any of those lists is non-empty, that means tags are present
    //    and we have separate robot-pose estimates for each seen tag.
    //    We add each estimate to the pose estimator as a vision measurement.
    frontPoses.forEach(pose -> 
        poseEstimator.addVisionMeasurement(pose, currentTime)
    );
    leftPoses.forEach(pose ->
        poseEstimator.addVisionMeasurement(pose, currentTime)
    );
    rightPoses.forEach(pose ->
        poseEstimator.addVisionMeasurement(pose, currentTime)
    );
  }

  /**
   * @return The current best-known pose of the robot on the field (X in meters, Y in meters),
   * as determined by fusing odometry with AprilTag measurements from all 3 Limelights.
   */
  public Pose2d getCurrentPoseOnField() 
  {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose estimate to the given pose.
   * For example, when you know exactly where you are on the field.
   */
  public void resetPose(Pose2d newPose) 
  {
    poseEstimator.resetPosition(
        getGyroHeading(),
        swerveDrivetrain.getModulePositions(),
        newPose
    );
  }

  /**
   * @return The AprilTagFieldLayout loaded from the 2025-reefscape.json file,
   *         which you can use to look up tag poses or field dimensions.
   */
  public AprilTagFieldLayout getFieldLayout() 
  {
    return fieldLayout;
  }
}
