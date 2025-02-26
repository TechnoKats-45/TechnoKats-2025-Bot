 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.generated.TunerConstants;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer 
{
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);              // Driver Controller
    private final CommandJoystick operator = new CommandJoystick(1);                        // Operator Button Board
    private final CommandXboxController testController = new CommandXboxController(2);      // Test Controller

    // Subsystems:
    public final Swerve s_swerve = TunerConstants.createDrivetrain();
    public final Carriage s_carriage = new Carriage();
    public final Elevator s_elevator = new Elevator();
    public final Climber s_climber = new Climber();
    public final Hopper s_hopper = new Hopper();

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
            // Drivetrain will execute this command periodically
            s_swerve.applyRequest
            (() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        */

        /*
        s_swerve.setDefaultCommand
        (
            s_swerve.applyRequest
            (() -> 
                drive.withVelocityX(-s_swerve.getLimitedXSpeed(driver.getLeftY(), MaxSpeed, s_elevator.getHeight()))    //-s_swerve.getLimitedXSpeed(driver.getLeftY(), MaxSpeed, s_elevator.getHeight())
                    .withVelocityY(-s_swerve.getLimitedYSpeed(driver.getLeftX(), MaxSpeed, s_elevator.getHeight())) //-s_swerve.getLimitedYSpeed(driver.getLeftX(), MaxSpeed, s_elevator.getHeight())
                    .withRotationalRate(-s_swerve.getLimitedRotSpeed(driver.getRightX(), MaxAngularRate, s_elevator.getHeight()))   //-s_swerve.getLimitedRotSpeed(driver.getRightX(), MaxAngularRate, s_elevator.getHeight())
            )
        );
        */

        s_swerve.setDefaultCommand
        (
            // Drivetrain will execute this command periodically
            s_swerve.applyRequest
            (() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        

        s_carriage.setDefaultCommand
        (
            // Carriage will execute this command periodically
            new CarriageDefault(s_carriage)
        );

        /*
        s_elevator.setDefaultCommand
        (
            // Elevator will execute this command periodically  // TODO - Comment out when tuned / tested
            new ManualElevator(s_elevator, testController)
        );
        */
        
        s_climber.setDefaultCommand // TODO - Comment out when tuned / tested - still need to get set points
        (
            new ManualClimber(s_climber, testController)
        );

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
        /// DRIVER CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        driver.leftTrigger().whileTrue(new GoToHeightPreset(s_elevator));
        driver.b().onTrue(s_swerve.runOnce(() -> s_swerve.seedFieldCentric()));             // B button - Reset the field-centric heading on B button press
        driver.rightBumper().onTrue(new CoralIntake(s_carriage, s_elevator));
        driver.rightTrigger().whileTrue(s_carriage.run(() -> s_carriage.setCoralSpeed(Constants.Carriage.coralScoreSpeed)));
        //driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));    // Start Button - Cancel All Commands
        driver.back().onTrue(s_swerve.runOnce(() -> s_swerve.poseToLL()));

        /*
        driver.povUp().onTrue
        (
            new ConditionalCommand
            (
                s_climber.runOnce(() -> s_climber.setAngle(s_climber.getAngle() + 5)), // Increase by 5°
                new InstantCommand(),  // Do nothing if false
                () -> s_climber.isClimbEnabled() // Only run if operator.button(0) is pressed / climb is enabled
            )
        );
        
        driver.povDown().onTrue
        (
            new ConditionalCommand
            (
                s_climber.runOnce(() -> s_climber.setAngle(s_climber.getAngle() - 5)), // Decrease by 5°
                new InstantCommand(),  // Do nothing if false
                () -> s_climber.isClimbEnabled() // Only run if operator.button(0) is pressed / climb is enabled
            )
        );
        */
        
        

        /*
        // Sequential - raise elevator once aligned to position
        driver.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                new GoToHeightPreset(s_carriage, s_elevator, s_swerve), // 2. Go to height
                new ConditionalCommand  // 3. Score / Clean
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

        // Parallel - raise elevator while alinging to position
        /*
        driver.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                new ParallelCommandGroup
                (
                    new PositionAlign(s_swerve, s_carriage, driver), // 1. Align to position
                    new ConditionalCommand  // Go to height if within X inches of target
                    (
                        // If the condition is TRUE, go to height
                        new GoToHeightPreset(s_elevator),
                        // Otherwise, run elevator default
                        new ElevatorDefault(s_elevator),
                        // The condition (must be a BooleanSupplier)
                        () -> s_swerve.isWithinTolerance(24) && s_swerve.isRotationComplete() // Check if within X inches of target and rotation is complete
                    )
                ),
                new ConditionalCommand  // 3. Score / Clean
                (
                    // If the condition is TRUE, run AutoClean
                    new AutoClean(s_carriage, s_elevator),  
                    // Otherwise, run AutoScore - passes if is set to Barge Height
                    new AutoScore(s_carriage, s_elevator, operator.button(Constants.Button.height.Barge).getAsBoolean()),   
                    // The condition (must be a BooleanSupplier)
                    () -> operator.button(Constants.Button.height.A1).getAsBoolean() || operator.button(Constants.Button.height.A2).getAsBoolean()  // Check if set to either A1 or A2 heights
                )
            )
        );*/

        /*  Add back in once SysID is completed
        // Start Button - Cancel All Commands
        driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        */        

        //////////////////////////////////////////////////////////////////////////////////////////
        /// OPERATOR BUTTON BOARD CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        operator.button(Constants.Button.height.L1).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L1)));
        operator.button(Constants.Button.height.L2).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L2)));
        operator.button(Constants.Button.height.L3).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L3)));
        operator.button(Constants.Button.height.L4).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L4)));
        operator.button(Constants.Button.height.Barge).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.Barge)));

        operator.button(Constants.Button.height.A1).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.A1)));
        operator.button(Constants.Button.height.A2).onTrue(new InstantCommand(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.A2)));

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

        operator.button(Constants.Button.H).onTrue(new InstantCommand(() -> s_climber.enableClimb()));

        //////////////////////////////////////////////////////////////////////////////////////////
        /// OPERATOR CONTROLLER / TEST CONTROLLER
        //////////////////////////////////////////////////////////////////////////////////////////
        testController.leftTrigger().whileTrue(new PositionAlign(s_swerve));


        //////////////////////////////////////////////////////////////////////////////////////////
        // SYSID ROUTINES
        //////////////////////////////////////////////////////////////////////////////////////////

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
            driver.back().and(driver.y()).whileTrue(s_swerve.sysIdDynamic(Direction.kForward));
            driver.back().and(driver.x()).whileTrue(s_swerve.sysIdDynamic(Direction.kReverse));
            driver.start().and(driver.y()).whileTrue(s_swerve.sysIdQuasistatic(Direction.kForward));
            driver.start().and(driver.x()).whileTrue(s_swerve.sysIdQuasistatic(Direction.kReverse));

            s_swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected();
    }

    public void registerEventTriggers()
    {
        // TODO
        //Example:
            // Use event markers as triggers
            new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    }

    public void printDiagnostics()
    {
        s_elevator.printDiagnostics();
        s_carriage.printDiagnostics();
        s_climber.printDiagnostics();
    }

    public void registerNamedCommands()
    {
        NamedCommands.registerCommand
        (
            "CoralStationIntake",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight), s_elevator),
                new HopperIntake(s_carriage, s_elevator, s_hopper)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-A",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.A)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-B",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.B)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-C",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.C)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-D",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.D)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-E",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.E)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-F",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.F)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-G",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.G)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-H",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.H)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-I",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.I)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-J",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.J)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-K",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.K)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-L",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.L)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-LeftCoral",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.LeftCoral)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-RightCoral",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.RightCoral)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-Barge",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.Barge)
            )
        );

        NamedCommands.registerCommand
        (
            "FinalAlignment-Processor",
            new ParallelCommandGroup
            (
                new PositionAlign(s_swerve, Constants.Destinations.Processor)
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL1",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator),
                new AutoScore(s_carriage, s_elevator, false)
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL2",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L2), s_elevator),
                new AutoScore(s_carriage, s_elevator, false)
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL3",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L3), s_elevator),
                new AutoScore(s_carriage, s_elevator, false)
            )
        );

        NamedCommands.registerCommand
        (
            "ScoreL4",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L4), s_elevator),
                new AutoScore(s_carriage, s_elevator, false)
            )
        );
        
        NamedCommands.registerCommand
        (
            "ScoreBarge",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.Barge), s_elevator),
                new AutoScore(s_carriage, s_elevator, true)
            )
        );

        NamedCommands.registerCommand
        (
            "CleanA1",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.A1), s_elevator),
                new AutoClean(s_carriage, s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "CleanA2",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.A2), s_elevator),
                new AutoClean(s_carriage, s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHeightL1",
            new SequentialCommandGroup
            (
                
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHeightL2",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHeightL3",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHeightL4",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
            )
        );

        NamedCommands.registerCommand
        (
            "GoToHeightBarge",
            new SequentialCommandGroup
            (
                new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
            )
        );
    }
}
