package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;

import frc.robot.Constants;

public class HopperIntake extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    
    private Hopper s_hopper;

    public HopperIntake(Carriage s_carriage, Elevator s_elevator, Hopper s_hopper)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;
        this.s_hopper = s_hopper;

        addRequirements(s_carriage, s_elevator);    // Probably don't need to require elevator since just reading from it
    }

    public void execute()
    {
        if(!s_carriage.isAlgaeDetected() && s_elevator.isAligned())
        {
            s_carriage.setAlgaeSpeed(Constants.Carriage.algaeCleanSpeed);
        }
        else
        {
            s_carriage.setAlgaeSpeed(0);
        }
    }

    public boolean isFinished()
    {
        return s_carriage.isAlgaeDetected() || s_hopper.isCoralDetected();
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setAlgaeSpeed(0);
    }
}
