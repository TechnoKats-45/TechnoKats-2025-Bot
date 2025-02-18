package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class AutoClean extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    

    public AutoClean(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;

        addRequirements(s_carriage);

        s_carriage.setAlgaeAngle(Constants.Carriage.algaeCleanAngle);
    }

    public void execute()
    {
        if (s_carriage.isAlgaeAngleAligned())
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
        return !s_carriage.isAlgaeDetected();
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setAlgaeSpeed(0);
        s_carriage.setAlgaeAngle(Constants.Carriage.algaeStowAngle);
    }
}
