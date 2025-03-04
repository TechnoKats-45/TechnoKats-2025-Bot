package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Swerve;

public class PositionAlign extends Command
{
    private Swerve s_swerve;
    private Pose2d targetPose;

    public PositionAlign(Swerve s_swerve)   // For Teleop
    {
        this.s_swerve = s_swerve;
    
        addRequirements(s_swerve);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute() 
    {
        targetPose = s_swerve.getDestination();

        if (targetPose == null) 
        {
            DriverStation.reportError("Destination pose is null!", false);
            return;
        }
    
        System.out.println("Executing PositionAlign to: " + targetPose);
    
        Command pathCommand = s_swerve.driveToPose(targetPose);
        if (pathCommand != null) 
        {
            pathCommand.schedule(); // Ensure the command runs!
        } 
        else 
        {
            DriverStation.reportError("Failed to generate pathfinding command!", false);
        }
    }
    

    @Override
    public boolean isFinished()
    {
        if(DriverStation.isTeleop())
        {
            return false;
        }
        if(DriverStation.isAutonomous())
        {
            return true;
        }
        else
        {
            System.out.println("Error: Not in Teleop or Autonomous = PositionAlign Failed");
            return false;
        }
    }
    
    @Override
    public void end(boolean interrupted)
    {
        
    }
}
