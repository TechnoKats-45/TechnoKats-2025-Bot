package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class FloorCoralIntake extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    

    public FloorCoralIntake(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;

        addRequirements(s_carriage);    // Probably don't need to require elevator since just reading from it
    }

    public void execute()
    {
        if(!s_carriage.isCoralDetected() && s_elevator.isAligned())
        {
            s_carriage.setCoralSpeed(Constants.Carriage.coralIntakeSpeed);
        }
        else
        {
            s_carriage.setCoralSpeed(0);
        }
    }

    public boolean isFinished()
    {
        return s_carriage.isCoralDetected();
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
    }
}
