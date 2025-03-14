package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;

public class ManualClimber extends Command
{
    private Climber s_climber;
    private CommandXboxController controller;
    private CommandXboxController driver;
    private final Joystick rumbleController;


    public ManualClimber(Climber s_climber, CommandXboxController controller, CommandXboxController driver, Joystick rumbleController)
    {
        this.s_climber = s_climber;
        this.controller = controller;
        this.driver = driver;
        this.rumbleController = rumbleController;

        addRequirements(s_climber);
    }

    @Override
    public void execute()
    {
        s_climber.ManualClimber(controller, driver);
        
        if(s_climber.isClimberAligned() && s_climber.isClimbEnabled())
        {
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 1); 
        }
        else
        {
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        }
    }
}
