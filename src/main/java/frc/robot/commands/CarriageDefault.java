package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Carriage;

public class CarriageDefault extends Command
{
    private Carriage s_Carriage;    

    public CarriageDefault(Carriage s_Carriage)
    {
        this.s_Carriage = s_Carriage;

        addRequirements(s_Carriage);
    }

    public void execute()
    {
        s_Carriage.setCoralSpeed(0);
        s_Carriage.setAlgaeAngle(0);
    }

    public boolean isFinished()
    {
        return false;
    }
    

    public void end(boolean interrupted)
    {
        
    }
}
