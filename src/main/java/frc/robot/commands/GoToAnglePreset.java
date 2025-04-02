package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class GoToAnglePreset extends Command
{
    private Elevator s_elevator;  
    private CommandXboxController driver;
    
    private boolean ranFlag;
    private double currentPreset;

    public GoToAnglePreset(Elevator s_elevator, Carriage s_carriage, CommandXboxController driver)
    {
        this.s_elevator = s_elevator;
        this.driver = driver;

        ranFlag = false;

        addRequirements(s_elevator);
    }

    public void execute()
    {
        if(!ranFlag)
        {
            currentPreset = s_elevator.getAnglePreset();
            s_elevator.setAngle(currentPreset);
            ranFlag = true;
        }
        else
        {
            if(driver.a().getAsBoolean())
            {
                currentPreset = s_elevator.getAnglePreset();
                s_elevator.setAngle(currentPreset);
            }
        }
    }

    public boolean isFinished()
    {
        //return s_elevator.isAligned();    // This 
        return false;
    }
    

    public void end(boolean interrupted)
    {
        ranFlag = false;
    }
}
