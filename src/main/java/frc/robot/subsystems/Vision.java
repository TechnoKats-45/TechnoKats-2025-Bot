package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase 
{
    private final NetworkTable limelightFront;
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;

    public Vision() 
    {
        limelightFront = NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_FRONT_TABLE);
        limelightLeft  = NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_LEFT_TABLE);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_RIGHT_TABLE);
    }

    /**
     * Returns the raw "botpose" array from the specified Limelight.
     * 
     * @param limelight "front", "left", or "right"
     * @return double[6] array: [X, Y, Z, roll, pitch, yaw] or empty array if invalid
     */
    public double[] getTargetPose(String limelight) 
    {
        NetworkTable table = getTable(limelight);
        return table.getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * Checks whether the specified Limelight currently has a valid target.
     * 
     * @param limelight "front", "left", or "right"
     * @return true if a valid target is detected
     */
    public boolean hasValidTarget(String limelight) 
    {
        NetworkTable table = getTable(limelight);
        NetworkTableEntry tvEntry = table.getEntry("tv");
        // 1.0 means valid target, 0.0 means no valid target
        return (tvEntry.getDouble(0.0) >= 1.0);
    }

    /**
     * Converts the "botpose" array into a WPILib Pose2d (x, y in meters, yaw in degrees).
     * 
     * If your pipeline or usage is different, adjust indexes or transform as appropriate.
     *
     * @param botpose double[6] array: [X, Y, Z, roll, pitch, yaw]
     * @return A Pose2d or a default (0,0,0°) pose if array is invalid
     */
    public Pose2d toPose2d(double[] botpose) 
    {
        if (botpose.length < 6) {
            return new Pose2d();
        }
        double x = botpose[0];
        double y = botpose[1];
        double yawDegrees = botpose[5];
        return new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
    }

    /**
     * Example: Retrieve the pipeline latency if needed, to help with time offsets
     * in your pose estimator.
     * 
     * @param limelight "front", "left", or "right"
     * @return The pipeline latency in milliseconds, or 0 if not found
     */
    public double getPipelineLatencyMillis(String limelight) 
    {
        NetworkTable table = getTable(limelight);
        return table.getEntry("tl").getDouble(0.0);
    }

    /**
     * Example of how you might automatically push vision data to a Localizer.
     * If you prefer, call these methods directly from Robot or a command.
     */
    @Override
    public void periodic() 
    {
        // (Optional) If you'd like to process all cameras automatically here, do so.
        // For example:
        // if (hasValidTarget("front")) {
        //     double[] poseArray = getTargetPose("front");
        //     Pose2d pose = toPose2d(poseArray);
        //     double timestamp = ... // compute or retrieve
        //     localizer.addVisionMeasurement(pose, timestamp, someStdDevs);
        // }
        // And repeat for "left" and "right".
    }

    /**
     * Helper to get the correct NetworkTable by camera name.
     * 
     * @param limelight "front", "left", or "right"
     * @return The corresponding NetworkTable
     */
    private NetworkTable getTable(String limelight) 
    {
        switch (limelight) {
            case "front":
                return limelightFront;
            case "left":
                return limelightLeft;
            case "right":
                return limelightRight;
            default:
                throw new IllegalArgumentException("Invalid Limelight name: " + limelight);
        }
    }

    
}
