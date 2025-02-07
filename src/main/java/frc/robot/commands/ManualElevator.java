package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command
{
    private Elevator s_Elevator;
    private CommandXboxController controller;

    public ManualElevator(Elevator s_Elevator, CommandXboxController controller)
    {
        this.s_Elevator = s_Elevator;
        this.controller = controller;

        addRequirements(s_Elevator);
    }

    @Override
    public void execute()
    {
        s_Elevator.ManualElevator(controller);
    }
}
