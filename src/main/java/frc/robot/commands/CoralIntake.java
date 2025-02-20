package frc.robot.commands;


import javax.xml.transform.stream.StreamResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class CoralIntake extends Command
{
    private Carriage s_carriage;
    private Elevator s_elevator;   
    private boolean coralHasBeenSeen = false;
    private boolean reverse = false;

    public CoralIntake(Carriage s_carriage, Elevator s_elevator)
    {
        this.s_carriage = s_carriage;
        this.s_elevator = s_elevator;

        addRequirements(s_carriage);

        s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight); // TODO
    }

    public void execute()
    {
        if (!s_carriage.isCoralDetected() && !coralHasBeenSeen) // no coral, never been seen = passive intake   // NOPE
        {
            s_carriage.setCoralSpeed(Constants.Carriage.coralPassiveIntakeSpeed);
            SmartDashboard.putNumber("Intake Stage", 1);
        }
        else if (s_carriage.isCoralDetected() && !reverse)  // yes coral, and not reversed yet = slow intake    //
        {
            coralHasBeenSeen = true;
            s_carriage.setCoralSpeed(Constants.Carriage.coralSlowIntakeSpeed);
            SmartDashboard.putNumber("Intake Stage", 2);
        }
        else if (!s_carriage.isCoralDetected() && coralHasBeenSeen) // no coral, was seen = reverse // YES[]\
        {
            reverse = true;
            s_carriage.setCoralSpeed(Constants.Carriage.coralReverseSpeed);
            SmartDashboard.putNumber("Intake Stage", 3);
        }
        else 
        {
            SmartDashboard.putNumber("Intake Stage", 0);
        }
        SmartDashboard.putBoolean("COral Seen?", coralHasBeenSeen);
        SmartDashboard.putBoolean("REVERSE", reverse);
    }

    public boolean isFinished()
    {
        return s_carriage.isCoralDetected() && coralHasBeenSeen && reverse; // End when coral is detected, after being seen, and after having reversed
    }
    

    public void end(boolean interrupted)
    {
        s_carriage.setCoralSpeed(0);
        SmartDashboard.putNumber("Intake Stage", -1);
    }
}
