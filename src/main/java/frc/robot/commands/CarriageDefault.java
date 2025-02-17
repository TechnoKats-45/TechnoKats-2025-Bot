package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;


public class CarriageDefault extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;    

    public CarriageDefault(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;

        addRequirements(s_carriage);

        s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight); // TODO
    }

    public void execute()
    {
        if(true)   // If algae is not detected, run the intake at passive intake speed // !s_carriage.isAlgaeDetected()
        {
            s_carriage.setCoralSpeed(Constants.Carriage.coralPassiveIntakeSpeed);
            //s_carriage.setCoralSpeedDumb(.10);    // TODO - Change to passive intake speed
        }    
        else    // If algae is detected, run the intake at active intake speed
        {
            s_carriage.setCoralSpeed(0);
        }
    }

    public boolean isFinished()
    {
        //return s_carriage.isAlgaeDetected();
        return false;
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
    }
}
