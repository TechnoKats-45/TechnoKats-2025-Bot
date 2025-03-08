package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class CleanAlgae extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator; 
    private double angle;   
    private boolean intookSuccessfully;

    public CleanAlgae(Carriage s_carriage, Elevator s_elevator, double angle)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;
        this.angle = angle;

        addRequirements(s_carriage, s_elevator);

        s_elevator.setAngle(angle);

        intookSuccessfully = false;
    }

    public void execute()
    {
        if(!s_elevator.isAligned()) // If elevator is not aligned, wait until it is
        {
            s_elevator.setAngle(angle);   // Set to either A1 or A2
        }
        else if (s_elevator.isAligned() && !s_carriage.isAlgaeAngleAligned())   // If elevator is aligned, but algae mech is not deploted
        {
            s_carriage.setAlgaeSpeed(Constants.Carriage.algaeCleanSpeed);       // Deploy algae mech
            s_carriage.setAlgaeAngle(Constants.Carriage.algaeCleanAngle);       // Spin intake wheels
        }
        else if (s_elevator.isAligned() && s_carriage.isAlgaeAngleAligned())    // If elevator is aligned and algae mech is deployed
        {
            s_carriage.setAlgaeSpeed(Constants.Carriage.algaeCleanSpeed);       // Spin intake wheels
        }
        else if ( s_carriage.isAlgaeDetected())                                 // If algae is detected
        {
            s_carriage.setAlgaeSpeed(0);                                  // Stop spinning intake wheels
            s_carriage.setAlgaeAngle(Constants.Carriage.algaeStowAngle);        // Stow algae mech
            new WaitCommand(.5);                                        // Wait for algae mech to stow before going down
            s_elevator.setAngle(Constants.Elevator.AnglePresets.handoffAngle);    // Lower elevator to handoff height
            intookSuccessfully = true;
        }
    }

    public boolean isFinished()
    {
        return intookSuccessfully;
    }
    
    public void end(boolean interrupted)
    {
        s_carriage.setAlgaeSpeed(0);
        s_carriage.setAlgaeAngle(Constants.Carriage.algaeStowAngle);
    }
}
