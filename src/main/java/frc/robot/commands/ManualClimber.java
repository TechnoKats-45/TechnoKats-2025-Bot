package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;

public class ManualClimber extends Command
{
    private Climber s_climber;
    private CommandXboxController controller;

    public ManualClimber(Climber s_climber, CommandXboxController controller)
    {
        this.s_climber = s_climber;
        this.controller = controller;

        addRequirements(s_climber);
    }

    @Override
    public void execute()
    {
        s_climber.ManualClimber(controller);
    }
}
