 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer 
{
    // Create a DigitalInput on DIO port 2
    private final DigitalInput buttonInput = new DigitalInput(2);
    Trigger buttonTrigger = new Trigger(buttonInput::get);    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);              // Driver Controller
    private final Joystick rumbleDriver = new Joystick(0);                                  // Rumble Controller for Driver Controller
    private final CommandJoystick operator = new CommandJoystick(1);                        // Operator Button Board
    private final CommandXboxController testController = new CommandXboxController(2);      // Test Controller

    // Subsystems:
    public final Swerve s_swerve = TunerConstants.createDrivetrain();
    public final Carriage s_carriage = new Carriage();
    public final Elevator s_elevator = new Elevator();
    public final Climber s_climber = new Climber();    

    // LEDs:
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    public LEDSubsystem getLEDSubsystem()
    {
        return ledSubsystem;
    }
    

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        registerNamedCommands();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() 
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        /// DEFAULT COMMANDS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*
        s_swerve.setDefaultCommand
        (
            s_swerve.applyRequest
            (
                () -> driver.x().getAsBoolean()  // This checks the button state continuously
                    ? forwardStraight
                        .withVelocityX(-driver.getLeftY() * MaxSpeed * s_swerve.getSpeedFactor(s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))   // Robot-centric forward/backward
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * s_swerve.getSpeedFactor(s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))   // Robot-centric strafe left/right
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Robot-centric rotation
                    : drive
                        .withVelocityX(-driver.getLeftY() * MaxSpeed * s_swerve.getSpeedFactor(s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))   // Field-centric forward/backward
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * s_swerve.getSpeedFactor(s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))   // Field-centric strafe left/right
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Field-centric rotation
            )
        );
        */

        s_swerve.setDefaultCommand
        (
            s_swerve.applyRequest(() -> 
            {
                /*
                // If left trigger is held, run auto-align logic:
                if (driver.leftTrigger().getAsBoolean()) 
                {
                    // Get the current robot pose
                    Pose2d robotPose = s_swerve.getState().Pose;
                    // Compute the closest reef face based on the robot's current pose.
                    Pose2d targetPose = new GetClosestFace(s_swerve).getClosestFace(); // This will return the closest reef face based on the current robot pose.
                    // Compute the error between the target pose and the current pose.
                    double errorX = targetPose.getTranslation().getX() - robotPose.getTranslation().getX();
                    double errorY = targetPose.getTranslation().getY() - robotPose.getTranslation().getY();
                    double errorTheta = targetPose.getRotation().getRadians() - robotPose.getRotation().getRadians();
                    
                    // Use a simple proportional controller (tune kP as needed)
                    double kP_linear = 1.0;  
                    double kP_angular = 1.0;
                    double velocityX = kP_linear * errorX;
                    double velocityY = kP_linear * errorY;
                    double rotationalRate = kP_angular * errorTheta;
                    
                    // Return a SwerveRequest that drives toward the target.
                    return drive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
                } 
                            */
                    // Otherwise, use the normal manual drive logic.
                    return driver.x().getAsBoolean() 
                        ? forwardStraight
                            .withVelocityX(-driver.getLeftY() * MaxSpeed * s_swerve.getSpeedFactor(
                                    s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))
                            .withVelocityY(-driver.getLeftX() * MaxSpeed * s_swerve.getSpeedFactor(
                                    s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))
                            .withRotationalRate(-driver.getRightX() * MaxAngularRate)
                        : drive
                            .withVelocityX(-driver.getLeftY() * MaxSpeed * s_swerve.getSpeedFactor(
                                    s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))
                            .withVelocityY(-driver.getLeftX() * MaxSpeed * s_swerve.getSpeedFactor(
                                    s_elevator.getAngle(), s_climber.isClimbEnabled(), s_elevator.isAligned()))
                            .withRotationalRate(-driver.getRightX() * MaxAngularRate);
                
            })
        );

        s_carriage.setDefaultCommand
        (
            new CarriageDefault(s_carriage)
        );
        
        s_climber.setDefaultCommand // TODO - Still need to get set points
        (
            new ManualClimber(s_climber, testController, driver, rumbleDriver)
        );
        
        /*
        s_elevator.setDefaultCommand
        (
            // Elevator will execute this command periodically  // TODO - Comment out when tuned / tested
            new ManualElevator(s_elevator, testController)
        );
        */

        //////////////////////////////////////////////////////////////////////////////////////////
        /// TEST CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
                
        /*
        testController.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                //new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                new GoToHeightPreset(s_elevator, () -> s_elevator.getHeightPreset()) // 2. Go to height
            
                /*new ConditionalCommand  // 3. Score / Clean
                (
                    // If the condition is TRUE, run AutoClean
                    new AutoClean(s_carriage, s_elevator),  
                    // Otherwise, run AutoScore - passes if is set to Barge Height
                    new AutoScore(s_carriage, s_elevator, operator.button(Constants.Button.height.Barge).getAsBoolean()),
                    // The condition (must be a BooleanSupplier)
                    () -> operator.button(Constants.Button.height.A1).getAsBoolean() || operator.button(Constants.Button.height.A2).getAsBoolean()  // Check if set to either A1 or A2 heights
                )
            )
        );
        */

        //////////////////////////////////////////////////////////////////////////////////////////
        /// ROBOT BUTTON BINDINGS
        //////////////////////////////////////////////////////////////////////////////////////////
        //buttonTrigger.onTrue(s_swerve.runOnce(() -> s_swerve.seedFieldCentric()));  // TODO - Need to change function - this should save heading and then heading should be used at start of auto.

        //////////////////////////////////////////////////////////////////////////////////////////
        /// DRIVER CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        driver.leftTrigger().whileTrue(new GoToAnglePreset(s_elevator, s_carriage, driver));       // Go to selected preset
        driver.leftTrigger().onFalse(new CoralIntake(s_carriage, s_elevator));             // When the left trigger is released, run Coral Intake (this will intake coral when the trigger is released after going to a preset)
        driver.b().onTrue(s_swerve.runOnce(() -> s_swerve.seedFieldCentric()));             // B button - Reset the field-centric heading on B button press
        driver.rightBumper().onTrue(new CoralIntake(s_carriage, s_elevator));               // Start Coral Intake
        driver.rightTrigger().whileTrue(s_carriage.run(() -> s_carriage.setCoralSpeed(Constants.Carriage.coralScoreSpeed, s_elevator)));    // Shoot coral
        driver.leftBumper().whileTrue(new CleanAlgae(s_carriage, s_elevator));
        driver.y().whileTrue(new RunCommand(() -> s_carriage.setAlgaeSpeed(Constants.Carriage.algaeScoreSpeed)));

        //driver.a().whileTrue(new LastMileAlignment(s_swerve));  // TODO - Test if this works - this is the last mile alignment w/o pose, just April Tag Alignment, and dead reckoning.
        //driver.a().whileTrue(s_swerve.driveToPose(new Pose2d(5.781,4.176,new Rotation2d()))); // THIS WORKS
        //driver.a().whileTrue(new PositionAlign(s_swerve));
        //driver.a().onFalse(new PositionAlign(s_swerve).cancel());

        driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));    // Start Button - Cancel All Commands
        
        //////////////////////////////////////////////////////////////////////////////////////////
        /// OPERATOR BUTTON BOARD CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        operator.button(Constants.Button.height.L1).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L1)));
        operator.button(Constants.Button.height.L2).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L2)));
        operator.button(Constants.Button.height.L3).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L3)));
        operator.button(Constants.Button.height.L4).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L4)));
        operator.button(Constants.Button.height.Barge).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.Barge)));

        operator.button(Constants.Button.height.A1).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.Stow)));  // Currently being used as ground pickup
        operator.button(Constants.Button.height.A2).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.A2)));

        operator.button(Constants.Button.location.A).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.A)));
        operator.button(Constants.Button.location.B).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.B)));
        operator.button(Constants.Button.location.C).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.C)));
        operator.button(Constants.Button.location.D).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.D)));
        operator.button(Constants.Button.location.E).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.E)));
        operator.button(Constants.Button.location.F).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.F)));
        operator.button(Constants.Button.location.G).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.G)));
        operator.button(Constants.Button.location.H).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.H)));
        operator.button(Constants.Button.location.I).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.I)));
        operator.button(Constants.Button.location.J).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.J)));
        operator.button(Constants.Button.location.K).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.K)));
        operator.button(Constants.Button.location.L).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.L)));

        operator.button(Constants.Button.location.LeftCoral).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.LeftCoral)));
        operator.button(Constants.Button.location.RightCoral).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.RightCoral)));

        operator.button(Constants.Button.location.Barge).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.Barge)));
        operator.button(Constants.Button.location.Processor).onTrue(new InstantCommand(() -> s_swerve.setDestination(Constants.Destinations.Processor)));

        //operator.button(Constants.Button.H).onTrue(new InstantCommand(() -> s_climber.enableClimb()));
        //operator.button(Constants.Button.H).onTrue(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.Stow)));
        operator.button(Constants.Button.H).whileTrue(new RunCommand(() -> s_elevator.GoToPreset()));
        operator.button(Constants.Button.H).onFalse(new InstantCommand(() -> s_climber.disableClimb()));
        operator.button(Constants.Button.H).onFalse(new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.handoffAngle)));

        operator.button(Constants.Button.H).onTrue(new SequentialCommandGroup
        (
            //new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.Stow)),
            new InstantCommand(() -> s_climber.enableClimb()),
            new ParallelDeadlineGroup
            (
                new WaitCommand(1.25),
                new RunCommand(() -> s_climber.setClimberSpeed(1), s_climber)
            ),
            new ParallelDeadlineGroup
            (
                new WaitCommand(1.25),
                new RunCommand(() -> s_climber.setClimberSpeed(-1), s_climber)         
            )
        ));
        
        operator.button(Constants.Button.H).whileTrue(new SequentialCommandGroup
        (
            new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.Stow)),
            new GoToAnglePreset(s_elevator, s_carriage, driver)
        ));

        //////////////////////////////////////////////////////////////////////////////////////////
        /// TEST CONTROLLER
        //////////////////////////////////////////////////////////////////////////////////////////
            //testController.leftTrigger().whileTrue(s_swerve.driveToPose(new Pose2d(4,1.5,new Rotation2d()))); // THIS WORKS
            //testController.leftTrigger().whileTrue(new PositionAlign(s_swerve));


        s_swerve.registerTelemetry(logger::telemeterize);           
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected();
    }

    public void registerEventTriggers()
    {
        new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    }

    public void printDiagnostics()
    {
        s_elevator.printDiagnostics();
        s_carriage.printDiagnostics();
        s_climber.printDiagnostics();
    }

    public void updateLEDs()
    {
        ledSubsystem.runPattern(LEDPattern.solid(Color.kGreen));
    }

    public void registerNamedCommands()
    {
        NamedCommands.registerCommand
        (
            "CoralStationIntake",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.handoffAngle), s_elevator),
                new CoralIntake(s_carriage, s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL1",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L1), s_elevator),
                new GoToAnglePreset(s_elevator, s_carriage, driver),
                new WaitCommand(.1),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(3),
                    new AutoScoreWithDeadline(s_carriage, s_elevator, false)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL2",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L2), s_elevator),
                new GoToAnglePreset(s_elevator, s_carriage, driver),
                new WaitCommand(.1),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(3),
                    new AutoScoreWithDeadline(s_carriage, s_elevator, false)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL3",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.L3), s_elevator),
                new GoToAnglePreset(s_elevator, s_carriage, driver),
                new WaitCommand(.1),

                new ParallelDeadlineGroup
                (
                    new WaitCommand(3),
                    new AutoScoreWithDeadline(s_carriage, s_elevator, false)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL4",
            new SequentialCommandGroup
            (
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.125),
                    new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.L4), s_elevator)
                ),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.5),    // .75
                    new AutoScoreWithDeadline(s_carriage, s_elevator, false)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreBarge",
            new SequentialCommandGroup
            (
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.125),
                    new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.Barge), s_elevator)
                ),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.5),
                    new AutoScoreWithDeadline(s_carriage, s_elevator, true)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "PassiveIntakeAlgae",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_carriage.setAlgaeSpeed(-0.5), s_carriage)
            )
        );

        NamedCommands.registerCommand
        (
            "CoralIntakeDoNothing",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_carriage.setCoralSpeed(0))
            )
        );

        NamedCommands.registerCommand
        (
            "GoToL4Height",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.L4), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToBargeHeight",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.Barge), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToA1Height",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.A1), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHandoffHeight",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.handoffAngle))
            )
        );

        NamedCommands.registerCommand
        (
            "poseToLL",
            new SequentialCommandGroup
            (
                new InstantCommand(() -> s_swerve.poseToLL())           
            )
        );
        

        NamedCommands.registerCommand
        (
            "CleanAlgaeA1",
            new SequentialCommandGroup
            (
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.5),
                    new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.A1), s_elevator)
                ),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.5),
                    new CleanAlgae(s_carriage, s_elevator)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "CleanAlgaeA2",
            new SequentialCommandGroup
            (
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.5),
                    new InstantCommand(() -> s_elevator.setAngle(Constants.Elevator.AnglePresets.A2), s_elevator)
                ),
                new ParallelDeadlineGroup
                (
                    new WaitCommand(.75),
                    new CleanAlgae(s_carriage, s_elevator)
                )
            )
        );

        NamedCommands.registerCommand
        (
            "AutoScoreWithDeadline",
            new ParallelDeadlineGroup
            (
                new WaitCommand(1),
                new AutoScoreWithDeadline(s_carriage, s_elevator, false)
            )
        );
    }
}
