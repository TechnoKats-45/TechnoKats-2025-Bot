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
        s_carriage.setCoralSpeed(Constants.Carriage.autoCoralScoreSpeed);
    }

    public boolean isFinished()
    {
        return false;
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
        s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight);
    }
}
