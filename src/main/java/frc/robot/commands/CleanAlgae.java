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

    public CleanAlgae(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;

        addRequirements(s_carriage, s_elevator);

        s_elevator.setAngle(angle);

        intookSuccessfully = false;
    }

    public void execute()
    {
        // Check to see if elevator is aligned first?
        s_carriage.setAlgaeSpeed(Constants.Carriage.algaeIntakeSpeed);               // Spin intake wheels// Deploy algae mech
        s_carriage.setAlgaeAngle(Constants.Carriage.AnglePresets.algaeCleanAngle);  // Deploy algae mech
    }

    public boolean isFinished()
    {
        return intookSuccessfully;
    }
    
    public void end(boolean interrupted)
    {
        s_carriage.setAlgaeSpeed(0);
        s_carriage.setAlgaeAngle(Constants.Carriage.AnglePresets.algaeStowAngle);
    }
}
