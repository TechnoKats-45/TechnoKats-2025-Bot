package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class GoToAnglePreset extends Command
{
    private Elevator s_elevator;    

    public GoToAnglePreset(Elevator s_elevator, Carriage s_carriage)
    {
        this.s_elevator = s_elevator;

        addRequirements(s_elevator);
    }

    public void execute()
    {
        double currentPreset = s_elevator.getAnglePreset();
        
        s_elevator.setAngle(currentPreset);
    }

    public boolean isFinished()
    {
        return s_elevator.isAligned();
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
