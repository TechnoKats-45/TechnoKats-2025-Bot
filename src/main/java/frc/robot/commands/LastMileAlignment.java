package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightHelpers;

public class LastMileAlignment extends Command 
{
    private final Swerve swerve;
    private static final double TARGET_DISTANCE_METERS = Units.feetToMeters(2.0);
    private static final double FINAL_FORWARD_DISTANCE_METERS = Units.feetToMeters(2.0);
    private static final double ALIGNMENT_TOLERANCE_METERS = 0.05;
    private static final double ALIGNMENT_SPEED = 0.3;
    private static final double ROTATION_TOLERANCE_DEGREES = 5.0;
    private static final double LATERAL_SHIFT_METERS = Units.feetToMeters(11.337687) / 2.0;

    private boolean moveLeft = false; // Placeholder, can be set externally
    private boolean moveForward = false;
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
    }

    @Override
    public void execute() 
    {
        System.out.println("Last Mile Alignment");
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
            
            // Compute distance and angle to the last known tag position
            double distanceToTag = Math.hypot(tagX, tagY);
            double angleToTag = Math.atan2(tagY, tagX);

            // Determine movement speed based on distance
            double speed = (distanceToTag > TARGET_DISTANCE_METERS + ALIGNMENT_TOLERANCE_METERS) ? ALIGNMENT_SPEED : 0.0;
            double rotationSpeed = (Math.abs(tagRotation) > Units.degreesToRadians(ROTATION_TOLERANCE_DEGREES)) ? 0.3 * Math.signum(tagRotation) : 0.0;
            
            // If within target distance, prepare to move forward
            if (distanceToTag <= TARGET_DISTANCE_METERS + ALIGNMENT_TOLERANCE_METERS) 
            {
                moveForward = true;
            }

            // Calculate new position based on alignment relative to the tag
            double finalX = speed * Math.cos(angleToTag);
            double finalY = speed * Math.sin(angleToTag);
            
            // Move forward the last 2 feet if alignment is complete
            if (moveForward) 
            {
                finalX += FINAL_FORWARD_DISTANCE_METERS * Math.cos(tagRotation);
                finalY += FINAL_FORWARD_DISTANCE_METERS * Math.sin(tagRotation);
            }

            // Move left or right based on boolean flag
            if (moveLeft) 
            {
                finalX += LATERAL_SHIFT_METERS * Math.cos(tagRotation + Math.PI / 2);
                finalY += LATERAL_SHIFT_METERS * Math.sin(tagRotation + Math.PI / 2);
            } 
            else 
            {
                finalX -= LATERAL_SHIFT_METERS * Math.cos(tagRotation + Math.PI / 2);
                finalY -= LATERAL_SHIFT_METERS * Math.sin(tagRotation + Math.PI / 2);
            }

            // Send the calculated position to the swerve drive
            swerve.driveToPose(new Pose2d
            (
                finalX,
                finalY,
                new Rotation2d(rotationSpeed)
            ));
        }
    }

    @Override
    public boolean isFinished() 
    {
        // Stop when the final forward movement is complete
        return moveForward;
    }

    @Override
    public void end(boolean interrupted) 
    {
        // Stop the robot and print completion message
        //swerve.stop();
        System.out.println("Last Mile Alignment Completed");
    }
}
