package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

public class setAutoPose extends Command
{
    private Swerve s_swerve;
    private boolean run;

    public setAutoPose(Swerve s_swerve)    // For Auto     // TODO - Make sure this doesn't break anything
    {
        this.s_swerve = s_swerve;
        
        addRequirements(s_swerve);
    }

    @Override
    public void initialize()
    {
        run = false;
    }

    @Override
    public void execute() 
    {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        {
             s_swerve.resetRotation(Rotation2d.kZero);    // was kzero
             s_swerve.poseToLL();
        }
        else
        {
             s_swerve.resetRotation(Rotation2d.k180deg);
             s_swerve.poseToLL();
        }
        run = true;
    }
    

    @Override
    public boolean isFinished()
    {
        return run;
    }
    
    @Override
    public void end(boolean interrupted)
    {
        
    }
}
