package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class AutoScore extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    
    private Boolean isBarge;

    public AutoScore(Carriage s_carriage, Elevator s_elevator, Boolean isBarge)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;
        this.isBarge = isBarge;

        addRequirements(s_carriage, s_elevator);

        if(isBarge)
        {
            s_carriage.setAlgaeAngle(Constants.Carriage.algaeScoreAngle);
        }
    }

    public void execute()
    {
        if(!isBarge)
        {
            s_carriage.setCoralSpeed(Constants.Carriage.coralScoreSpeed);
        }
        else
        {
            if (s_carriage.isAlgaeAngleAligned())
            {
                s_carriage.setAlgaeSpeed(Constants.Carriage.algaeScoreSpeed); 
            }
            else 
            {
                s_carriage.setAlgaeSpeed(0);

            }
        }
    }

    public boolean isFinished()
    {
        if(isBarge) // If Algae no longer detected
        {
            return !s_carriage.isAlgaeDetected();
        }
        else    // If Coral no longer detected
        {
            return !s_carriage.isCoralDetected();
        }
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
        s_carriage.setAlgaeAngle(Constants.Carriage.algaeStowAngle);
        s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight);
    }
}
