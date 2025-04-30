package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignTest extends Command
{
    private Swerve s_swerve;    

    public AlignTest(Swerve s_swerve)
    {
        this.s_swerve = s_swerve;

        addRequirements(s_swerve);
    }

    public void execute()
    {
        s_swerve.driveToPose(new Pose2d(5.781,4.176,new Rotation2d()));
    }

    public boolean isFinished()
    {
        return false;
    }
    
    public void end(boolean interrupted)
    {
        
    }
}
