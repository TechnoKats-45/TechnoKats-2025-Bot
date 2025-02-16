package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;
import frc.robot.Constants;


public class ElevatorDefault extends Command
{
    private Elevator s_elevator;    

    boolean coralSeen;

    public ElevatorDefault(Elevator s_elevator)
    {
        this.s_elevator = s_elevator;

        addRequirements(s_elevator);
    }

    public void execute()
    {
        s_elevator.setAngle(Constants.Elevator.HeightPresets.handoffHeight);
    }

    public boolean isFinished()
    {
        return false;
    }
    

    public void end(boolean interrupted)
    {

    }
}
