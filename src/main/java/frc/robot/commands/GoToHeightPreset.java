package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;

public class GoToHeightPreset extends Command
{
    private Elevator s_elevator;    

    public GoToHeightPreset(Elevator s_elevator)
    {
        this.s_elevator = s_elevator;

        addRequirements(s_elevator);
    }

    public void execute()
    {
        if(true)    // TODO - was s_carriage.isCoralDetected()
        {
            s_elevator.GoToPreset();
        }
        else
        {
            s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.handoffHeight);
        }
    }

    public boolean isFinished()
    {
        //return s_elevator.isAligned();
        return false;
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
