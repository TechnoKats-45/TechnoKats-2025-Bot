package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class PositionAlign extends Command
{
    private Swerve s_swerve;
    private Pose2d targetPose;

    public PositionAlign(Swerve s_swerve, Carriage s_carriage, Elevator s_elevator)
    {
        this.s_swerve = s_swerve;
        
        addRequirements(s_swerve, s_carriage, s_elevator);
    }

    public void execute()
    {
        targetPose = s_swerve.getDestination();
        Pose2d currentPose = s_swerve.getState().Pose;

        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(targetPose.getX(), targetPose.getY())), targetPose.getRotation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath
        (
            waypoints, 
            new PathConstraints
            (
                4.0, 4.0, // TODO - check these numbers
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, targetPose.getRotation())
        );

        // Flips the path for the RED alliance.
        path.preventFlipping = false;

        AutoBuilder.followPath(path).schedule();
    }

    public boolean isFinished()
    {
        Pose2d currentPose = s_swerve.getState().Pose;
        Pose2d targetPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
    
        double positionTolerance = Units.inchesToMeters(1.5); // meters
        double rotationTolerance = Units.degreesToRadians(1.0); // radians
    
        boolean positionAligned = currentPose.getTranslation().getDistance(targetPose.getTranslation()) < positionTolerance;
        boolean rotationAligned = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < rotationTolerance;
    
        return positionAligned && rotationAligned;
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
