package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GetClosestFace extends Command 
{
    private final Swerve s_swerve;
    private Pose2d closestFace;

    // Define the reef faces as the poles from Constants.Destinations.
    // You can add additional destinations as needed.
    private static final Pose2d[] REEF_FACES = 
    {
        Constants.Destinations.A,
        Constants.Destinations.B,
        Constants.Destinations.C,
        Constants.Destinations.D,
        Constants.Destinations.E,
        Constants.Destinations.F,
        Constants.Destinations.G,
        Constants.Destinations.H,
        Constants.Destinations.I,
        Constants.Destinations.J,
        Constants.Destinations.K,
        Constants.Destinations.L
    };

    /**
     * Creates a new GetClosestFace command.
     * @param swerve The Swerve subsystem to obtain the current robot pose.
     */
    public GetClosestFace(Swerve swerve) 
    {
        this.s_swerve = swerve;
        addRequirements();
    }

    @Override
    public void initialize() 
    {
        // Get the current robot pose from your swerve drivetrain.
        Pose2d robotPose = s_swerve.getState().Pose;
        // Find the reef face (pole) closest to the current robot pose.
        closestFace = findClosestFace(robotPose);
        System.out.println("Closest reef face: " + closestFace);
    }

    /**
     * Iterates through the predefined reef face poses and returns the one closest to the given robot pose.
     * @param robotPose The current pose of the robot.
     * @return The Pose2d of the closest reef face.
     */
    private Pose2d findClosestFace(Pose2d robotPose) 
    {
        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;
        for (Pose2d face : REEF_FACES) 
        {
            double distance = robotPose.getTranslation().getDistance(face.getTranslation());
            if (distance < minDistance) 
            {
                minDistance = distance;
                closest = face;
            }
        }
        return closest;
    }

    @Override
    public boolean isFinished()
    {
        // Finish immediately after calculation.
        return true;
    }

    /**
     * Optional getter so that other parts of your code can access the result.
     */
    public Pose2d getClosestFace() 
    {
        return closestFace;
    }
}
