package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

import frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class AutoScoreWithDeadline extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;   
    private Boolean isBarge; 

    public AutoScoreWithDeadline(Carriage s_carriage, Elevator s_elevator, Boolean isBarge)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;
        this.isBarge = isBarge;

        addRequirements(s_carriage);

        if(isBarge)
        {
            s_carriage.setAlgaeAngle(Constants.Carriage.algaeScoreAngle);
        }
    }

    public void execute()
    {
        if(!isBarge)    // Is not barge - scoring coral
        {
            s_carriage.setCoralSpeed(Constants.Carriage.autoCoralScoreSpeed);
        }
        else            // Is barge - scoring algae
        {
            s_carriage.setCoralSpeed(Constants.Carriage.algaeScoreSpeed);
        }
    
    }

    public boolean isFinished()
    {
        return false;

        /*
        if(!isBarge)// If Scoring Coral
        {
            return !s_carriage.isCoralDetected();
        }
        else        // If Scoring Algae
        {
            return !s_carriage.isAlgaeDetected();
        }
        */
    }

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
        s_carriage.setAlgaeSpeed(0);
        s_carriage.setAlgaeAngle(Constants.Carriage.algaeStowAngle);
        s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight);
    }
}
