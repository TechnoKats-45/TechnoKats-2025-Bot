// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.net.ssl.SNIMatcher;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  public Robot() 
  {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    m_robotContainer.printDiagnostics();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) 
    {
      var driveState = m_robotContainer.s_swerve.getState();
      double headingDeg = m_robotContainer.s_swerve.getPigeon2().getRotation2d().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      SmartDashboard.putNumber("HEADING", headingDeg);
      
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) 
      {
        //m_robotContainer.s_swerve.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds); // Effects rotation

        // Get current robot pose
        Pose2d currentPose = m_robotContainer.s_swerve.getState().Pose;

        // Get vision-based pose, but only use X and Y, not rotation
        var visionPose = llMeasurement.pose;

        // Create a new pose that keeps the robot's original heading
        var filteredPose = new Pose2d
        (
          visionPose.getX(),  // Use vision-based X
          visionPose.getY(),  // Use vision-based Y
          currentPose.getRotation()  // Keep the current heading
        );

        // Apply vision update with the filtered pose
        m_robotContainer.s_swerve.addVisionMeasurement(filteredPose, llMeasurement.timestampSeconds);

        // ADD GYRO CORRECTION WITH STDEV CHECK
        double visionHeading = visionPose.getRotation().getDegrees();
        double gyroHeading = m_robotContainer.s_swerve.getPigeon2().getRotation2d().getDegrees();
        
        // Retrieve standard deviation of the Limelight data
        double[] stddevs = NetworkTableInstance.getDefault()
          .getTable("limelight")
          .getEntry("stddevs")
          .getDoubleArray(new double[6]);

        double avgStdDev = (stddevs[0] + stddevs[1] + stddevs[5]) / 3; // Average X, Y, and Rotation stddev

        // Log gyro vs. vision heading for debugging
        SmartDashboard.putNumber("HEADING (Gyro)", gyroHeading);
        SmartDashboard.putNumber("HEADING (Vision)", visionHeading);
        SmartDashboard.putNumber("HEADING DIFF", Math.abs(gyroHeading - visionHeading));
        SmartDashboard.putNumber("Vision StDev", avgStdDev);

        // Only correct the gyro if:
        // - The error between gyro and vision heading is significant (>5°)
        // - The robot is NOT turning quickly (omegaRps < 0.5)
        // - The vision data is HIGHLY RELIABLE (stdev < 0.5 meters)
        if (Math.abs(gyroHeading - visionHeading) > 5.0 && Math.abs(omegaRps) < 0.5 && avgStdDev < 0.5)  
        {
          // Reset gyro to match vision heading
          m_robotContainer.s_swerve.getPigeon2().setYaw(visionHeading);
        }
      }
    }

    /*
    if (kUseLimelight) 
    {
      var driveState = m_robotContainer.s_swerve.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // Limelight names
      String[] limelights = {"limelight-left", "limelight-right", "limelight-front"};  //{"limelightBackLeft", "limelightBackRight", "limelightFront"};

      // Place to store standard deviations of the limelights' readings
      double limelightAvgStdDev[] = new double[limelights.length];

      // Constants for dynamic standard deviation threshold
      double baseStdDev = 0.2; // Base standard deviation at close range
      double k_d = 0.05;       // Scaling factor for distance
      double k_t = 0.1;        // Reward for additional tags

      // Process each Limelight
      for (int i = 0; i < limelights.length; i++) 
      {
        String limelightName = limelights[i];

        // Calculate the average standard deviation for this Limelight
        double[] stddevs = NetworkTableInstance.getDefault()
          .getTable(limelightName)
          .getEntry("stddevs")
          .getDoubleArray(new double[6]);

        limelightAvgStdDev[i] = (stddevs[0] + stddevs[1] + stddevs[5]) / 3;

        // Retrieve vision measurement from the Limelight
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (llMeasurement != null && llMeasurement.tagCount > 0) 
        {
          // Calculate dynamic standard deviation threshold
          double dynamicThreshold = baseStdDev + (k_d * llMeasurement.avgTagDist) - (k_t * llMeasurement.tagCount);

          // Check conditions: dynamic threshold and angular velocity
          if (limelightAvgStdDev[i] < dynamicThreshold && Math.abs(omegaRps) < 2.0) 
          {
            LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
            m_robotContainer.s_swerve.addVisionMeasurement
            (
              llMeasurement.pose, 
              Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds)
            );
          }
        }
      }
    }*/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
