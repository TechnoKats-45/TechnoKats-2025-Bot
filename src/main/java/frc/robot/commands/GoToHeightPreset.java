package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;

public class GoToHeightPreset extends Command
{
    private Elevator s_elevator;    

    public GoToHeightPreset(Elevator s_elevator)
    {
        this.s_elevator = s_elevator;

        addRequirements(s_elevator);
    }

    public void execute()
    {
        double currentPreset = s_elevator.getHeightPreset();
        if(true)    // if coral detected
        {
            s_elevator.setHeight(currentPreset);
        }
        else
        {
            s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.handoffHeight);
        }
    }

    public boolean isFinished()
    {
        //return s_elevator.isAligned();
        return false;
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
