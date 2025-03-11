package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class AlgaeIntake extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    

    public AlgaeIntake(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;

        addRequirements(s_carriage);    // Probably don't need to require elevator since just reading from it
    }

    public void execute()
    {
        if(s_elevator.isAligned())  // && !s_carriage.isAlgaeDetected()
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
        //return s_carriage.isAlgaeDetected();
        return false;
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setAlgaeSpeed(0);
    }
}
