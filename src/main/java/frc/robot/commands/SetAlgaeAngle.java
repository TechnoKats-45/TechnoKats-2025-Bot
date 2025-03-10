package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class SetAlgaeAngle extends Command
{
    private CommandXboxController controller;
    private Carriage s_carriage;

    public SetAlgaeAngle(Carriage s_carriage, CommandXboxController controller)
    {
        this.controller = controller;
        this.s_carriage = s_carriage;

        addRequirements(s_carriage);
    }

    public void execute()
    {
        s_carriage.ManualAlgaeAngle(controller);
        s_carriage.setCoralSpeed(0);
    }
}
