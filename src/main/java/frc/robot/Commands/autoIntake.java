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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class autoIntake extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;

    public autoIntake(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;

        addRequirements(s_carriage);    // Probably don't need to require elevator since just reading from it
    }

    public void execute()
    {
        
    }

    public boolean isFinished()
    {
        return false;
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
