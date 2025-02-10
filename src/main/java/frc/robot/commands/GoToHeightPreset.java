package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;

public class GoToHeightPreset extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    
    private Swerve s_swerve;

    public GoToHeightPreset(Carriage s_carriage, Elevator s_elevator, Swerve s_swerve)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;

        addRequirements(s_carriage, s_elevator);
    }

    public void execute()
    {
        if(s_carriage.isCoralDetected())
        {
            s_elevator.setAngle();  // TODO - Change to height once implemented
        }
        else
        {
            s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.handoffHeight);
        }
    }

    public boolean isFinished()
    {
        return s_elevator.isAligned();
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
