package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase 
{
  // NetworkTable references for each Limelight
  private final NetworkTable limelightFront =
      NetworkTableInstance.getDefault().getTable("limelight-front");
  private final NetworkTable limelightLeft =
      NetworkTableInstance.getDefault().getTable("limelight-left");
  private final NetworkTable limelightRight =
      NetworkTableInstance.getDefault().getTable("limelight-right");

  public Vision() 
  {
    // Configure pipelines, LED modes, etc., if needed
  }

  // -----------------------------------------------------------
  //  1) HELPER TO READ MULTIPLE APRILTAG POSES FROM A LIMELIGHT
  // -----------------------------------------------------------
  /**
   * Returns a list of all detected AprilTag poses (in robot space) from the
   * given Limelight, based on the "botpose_targets" array. Each tag is stored
   * in sets of 6 numbers: [x, y, z, rollDeg, pitchDeg, yawDeg].
   *
   * @param limelight NetworkTable for the particular Limelight
   * @return List of Pose2d for each detected tag (ignoring pitch/roll).
   */
  private List<Pose2d> getAllTagPoses2d(NetworkTable limelight) 
  {
    // "botpose_targets" is an array of length = 6 * (number_of_tags_detected).
    double[] rawPoseArray = limelight.getEntry("botpose_targets").getDoubleArray(new double[0]);
    List<Pose2d> results = new ArrayList<>();

    // Each tag's pose is 6 elements: (x, y, z, rollDeg, pitchDeg, yawDeg)
    // If no tags are detected, rawPoseArray could be empty.
    for (int i = 0; i + 5 < rawPoseArray.length; i += 6)
    {
      double x = rawPoseArray[i + 0];
      double y = rawPoseArray[i + 1];
      // double z = rawPoseArray[i + 2]; // If needed in 3D
      double rollDeg = rawPoseArray[i + 3];
      double pitchDeg = rawPoseArray[i + 4];
      double yawDeg = rawPoseArray[i + 5];

      // Convert yaw to WPILib Rotation2d
      Rotation2d heading = Rotation2d.fromDegrees(yawDeg);

      // We ignore z, roll, pitch for a Pose2d
      results.add(new Pose2d(x, y, heading));
    }

    return results;
  }

  // ----------------------------------------------------------
  //  2) HELPER TO READ THE TAG ID ARRAY FROM A LIMELIGHT
  // ----------------------------------------------------------
  /**
   * Returns the array of tag IDs currently recognized by the given Limelight.
   * If multiple tags are seen, this array will have multiple entries.
   *
   * @param limelight NetworkTable for the particular Limelight
   * @return int[] of AprilTag IDs
   */
  private int[] getTagIDs(NetworkTable limelight) 
  {
    double[] tidRaw = limelight.getEntry("tid").getDoubleArray(new double[0]);
    int[] ids = new int[tidRaw.length];
    for (int i = 0; i < tidRaw.length; i++) 
    {
      ids[i] = (int) tidRaw[i];
    }
    return ids;
  }

  // ----------------------------------------------------------------
  //  3) PUBLIC METHODS - READ ALL TAG POSES OR IDS FROM EACH LIMELIGHT
  // ----------------------------------------------------------------
  public List<Pose2d> getFrontLimelightAllTagPoses2d() 
  {
    return getAllTagPoses2d(limelightFront);
  }

  public List<Pose2d> getLeftLimelightAllTagPoses2d() 
  {
    return getAllTagPoses2d(limelightLeft);
  }

  public List<Pose2d> getRightLimelightAllTagPoses2d() 
  {
    return getAllTagPoses2d(limelightRight);
  }

  public int[] getFrontLimelightTagIDs() 
  {
    return getTagIDs(limelightFront);
  }

  public int[] getLeftLimelightTagIDs() 
  {
    return getTagIDs(limelightLeft);
  }

  public int[] getRightLimelightTagIDs() 
  {
    return getTagIDs(limelightRight);
  }

  // -------------------------------------------------------------------------
  //  4) OPTIONAL: STILL KEEP THE OLD SINGLE 'botpose' IF YOU WANT A SINGLE POSE
  // -------------------------------------------------------------------------
  /**
   * For some Limelight pipelines, "botpose" is a single best-fit pose if
   * multiple tags are visible. This returns that single solution.
   */
  public double[] getRawBotPose(NetworkTable limelight) 
  {
    NetworkTableEntry botPoseEntry = limelight.getEntry("botpose");
    double[] arr = botPoseEntry.getDoubleArray(new double[0]);
    if (arr.length < 6) 
    {
      return null;
    }
    return arr;
  }

  // Example: front-limelight single best-fit pose
  public double[] getFrontLimelightRawBotPose() 
  {
    return getRawBotPose(limelightFront);
  }

  // Periodic logging or debug if needed
  @Override
  public void periodic() 
  {
    // Example: log how many tags each camera sees
    int frontCount = getFrontLimelightTagIDs().length;
    int leftCount  = getLeftLimelightTagIDs().length;
    int rightCount = getRightLimelightTagIDs().length;

    // System.out.printf("Front: %d tags, Left: %d tags, Right: %d tags\n",
    //                   frontCount, leftCount, rightCount);
  }
}
