package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class LastMileAlignment extends Command 
{
    private final Swerve swerve;
    private static final double TARGET_DISTANCE_METERS = Units.feetToMeters(2.0);  // Align at 2 feet from tag
    private static final double FINAL_FORWARD_DISTANCE_METERS = Units.feetToMeters(2.0); // Move forward 2 feet
    private static final double ALIGNMENT_TOLERANCE_METERS = 0.05; // Tolerance for distance
    private static final double ALIGNMENT_SPEED = 0.3; // Speed when aligning
    private static final double ROTATION_TOLERANCE_DEGREES = 5.0; // Rotation tolerance
    private static final double LATERAL_SHIFT_METERS = Units.feetToMeters(12.937677) / 2.0; // Hardcoded lateral offset

    private boolean moveRight = true; // Hardcoded to always go right
    private boolean moveForward = false;
    private boolean lateralShiftComplete = false;
    private Pose2d lastKnownTagPose = null;

    public LastMileAlignment(Swerve swerve) 
    {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() 
    {
        System.out.println("Starting Last Mile Alignment");
        lastKnownTagPose = null;
        moveForward = false;
        lateralShiftComplete = false;
    }

    @Override
    public void execute() 
    {
        System.out.println("Executing Last Mile Alignment...");

        // Get AprilTag position from Limelight
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (llMeasurement != null && llMeasurement.tagCount > 0) 
        {
            // Store the last known position of the AprilTag
            lastKnownTagPose = llMeasurement.pose;
        }

        if (lastKnownTagPose != null) 
        {
            double tagX = lastKnownTagPose.getX();
            double tagY = lastKnownTagPose.getY();
            double tagRotation = lastKnownTagPose.getRotation().getRadians();

            // Compute distance to the tag
            double distanceToTag = Math.hypot(tagX, tagY);
            double angleToTag = Math.atan2(tagY, tagX);

            // Compute rotation speed to face the tag
            double rotationSpeed = (Math.abs(tagRotation) > Units.degreesToRadians(ROTATION_TOLERANCE_DEGREES)) ? 0.3 * Math.signum(tagRotation) : 0.0;

            if (!moveForward) 
            {
                // Align perpendicular to the tag
                double forwardSpeed = (distanceToTag > TARGET_DISTANCE_METERS + ALIGNMENT_TOLERANCE_METERS) ? ALIGNMENT_SPEED : 0.0;

                if (distanceToTag <= TARGET_DISTANCE_METERS + ALIGNMENT_TOLERANCE_METERS) 
                {
                    moveForward = true; // Start moving forward after alignment
                }

                // Drive toward the target distance
                SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                    .withVelocityX(forwardSpeed * Math.cos(angleToTag))
                    .withVelocityY(forwardSpeed * Math.sin(angleToTag))
                    .withRotationalRate(rotationSpeed);

                swerve.setControl(request);
                System.out.println("Aligning - SpeedX: " + forwardSpeed * Math.cos(angleToTag) + " SpeedY: " + forwardSpeed * Math.sin(angleToTag) + " Rotation: " + rotationSpeed);
            } 
            else if (!lateralShiftComplete) 
            {
                // Move forward the last 2 feet
                SwerveRequest.RobotCentric forwardRequest = new SwerveRequest.RobotCentric()
                    .withVelocityX(FINAL_FORWARD_DISTANCE_METERS) 
                    .withVelocityY(0)
                    .withRotationalRate(0);

                swerve.setControl(forwardRequest);
                System.out.println("Moving Forward");

                lateralShiftComplete = true;
            } 
            else 
            {
                // Shift laterally to the right
                double lateralX = moveRight ? LATERAL_SHIFT_METERS * Math.cos(tagRotation + Math.PI / 2) : -LATERAL_SHIFT_METERS * Math.cos(tagRotation + Math.PI / 2);
                double lateralY = moveRight ? LATERAL_SHIFT_METERS * Math.sin(tagRotation + Math.PI / 2) : -LATERAL_SHIFT_METERS * Math.sin(tagRotation + Math.PI / 2);

                SwerveRequest.RobotCentric lateralRequest = new SwerveRequest.RobotCentric()
                    .withVelocityX(lateralX)
                    .withVelocityY(lateralY)
                    .withRotationalRate(0);

                swerve.setControl(lateralRequest);
                System.out.println("Shifting Right");
            }
        }
        else 
        {
            System.out.println("No AprilTag Detected!");
        }
    }

    @Override
    public boolean isFinished() 
    {
        return lateralShiftComplete;
    }

    @Override
    public void end(boolean interrupted) 
    {
        swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0)); // Stop
        System.out.println("Last Mile Alignment Completed. Interrupted? " + interrupted);
    }
}
