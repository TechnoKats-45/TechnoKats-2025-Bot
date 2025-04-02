package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberPrep extends Command
{
    private Climber s_climber;    

    public ClimberPrep(Climber s_climber)
    {
        this.s_climber = s_climber;

        addRequirements(s_climber);
    }

    public void execute()
    {
        s_climber.setClimberSpeed();

    }

    public boolean isFinished()
    {
        return false;
    }
    
    public void end(boolean interrupted)
    {
        
    }
}
