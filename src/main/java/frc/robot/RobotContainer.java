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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);              // Driver Controller
    private final CommandJoystick operator = new CommandJoystick(1);                        // Operator Button Board
    private final CommandXboxController testController = new CommandXboxController(2);      // Test Controller

    // Subsystems:
    public final Swerve s_swerve = TunerConstants.createDrivetrain();
    public final Carriage s_carriage = new Carriage();
    public final Elevator s_elevator = new Elevator();
    public final Climber s_climber = new Climber();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

        registerNamedCommands();
        configureBindings();
    }

    private void configureBindings() 
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        /// DEFAULT COMMANDS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
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
            new CarriageDefault(s_carriage, s_elevator)
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
        
        //testController.leftTrigger().onTrue(s_elevator.runOnce(() -> s_elevator.setAngle(16)));   // 8.02 deg
        //testController.leftBumper().onTrue(s_elevator.runOnce(() -> s_elevator.setHeight(8+24)));      // 12 deg
        //testController.rightBumper().onTrue(s_elevator.runOnce(() -> s_elevator.setHeight(8+24+24)));     // 16 deg
        //testController.rightTrigger().onTrue(s_elevator.runOnce(() -> s_elevator.setHeight(79)));    // 19 deg
        //testController.a().onTrue(s_elevator.runOnce(() -> s_elevator.determineKs()));
        
        
        testController.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                //new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                new GoToHeightPreset(s_carriage, s_elevator, s_swerve) // 2. Go to height
                /*new ConditionalCommand  // 3. Score / Clean
                (
                    // If the condition is TRUE, run AutoClean
                    new AutoClean(s_carriage, s_elevator),  
                    // Otherwise, run AutoScore - passes if is set to Barge Height
                    new AutoScore(s_carriage, s_elevator, operator.button(Constants.Button.height.Barge).getAsBoolean()),
                    // The condition (must be a BooleanSupplier)
                    () -> operator.button(Constants.Button.height.A1).getAsBoolean() || operator.button(Constants.Button.height.A2).getAsBoolean()  // Check if set to either A1 or A2 heights
                )*/
            )
        );

        /*
        // TODO - Test if this works
        // Fixing the blocking issue
        // Parallel - raise elevator while alinging to position
        testController.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                new ParallelCommandGroup
                (
                    new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                    new ConditionalCommand  // Go to height if within X inches of target
                    (
                        // If the condition is TRUE, go to height
                        new GoToHeightPreset(s_carriage, s_elevator, s_swerve),  
                        // Otherwise, run elevator default
                        new ElevatorDefault(s_elevator),
                        // The condition (must be a BooleanSupplier)
                        () -> s_swerve.isWithinTolerance(24) && s_swerve.isRotationComplete() // Check if within X inches of target and rotation is complete
                    )
                ),
                new ParallelCommandGroup
                (
                    new ConditionalCommand  // 3. Score / Clean // Potnetially need to change to "either" command
                    (
                        // If the condition is TRUE, run AutoClean
                        new AutoClean(s_carriage, s_elevator),  
                        // Otherwise, run AutoScore - passes if is set to Barge Height
                        new AutoScore(s_carriage, s_elevator, operator.button(Constants.Button.height.Barge).getAsBoolean()),
                        // The condition (must be a BooleanSupplier)
                        () -> operator.button(Constants.Button.height.A1).getAsBoolean() || operator.button(Constants.Button.height.A2).getAsBoolean()  // Check if set to either A1 or A2 heights
                    ),
                    // go to height - for if the operator changes preset at this point
                    new GoToHeightPreset(s_carriage, s_elevator, s_swerve)
                )
            )
        );   
        */     

        /*
        // TODO - Test if this works
        // Fixing the blocking issue
        // Parallel - raise elevator while aligning to position
        testController.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                new ParallelCommandGroup
                (
                    new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                    new SelectCommand<>  // Dynamically selects between GoToHeightPreset or ElevatorDefault
                    (
                        Map.of
                        (
                            true, new GoToHeightPreset(s_carriage, s_elevator, s_swerve),
                            false, new ElevatorDefault(s_elevator)
                        ),
                        () -> s_swerve.isWithinTolerance(24) && s_swerve.isRotationComplete() // Live condition check
                    )
                ),
                new SelectCommand<>  // Dynamically selects between AutoClean and AutoScore
                (
                    Map.of
                    (
                        true, new AutoClean(s_carriage, s_elevator),  
                        false, new AutoScore(s_carriage, s_elevator, operator.button(Constants.Button.height.Barge).getAsBoolean())
                    ),
                    () -> operator.button(Constants.Button.height.A1).getAsBoolean() || operator.button(Constants.Button.height.A2).getAsBoolean() // Live condition check
                )
            )
        );
        */


        //////////////////////////////////////////////////////////////////////////////////////////
        /// DRIVER CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////

        driver.a().whileTrue(s_swerve.applyRequest(() -> brake));                           // A button - brake the drivetrain
        driver.b().onTrue(s_swerve.runOnce(() -> s_swerve.seedFieldCentric()));             // B button - Reset the field-centric heading on B button press
        driver.povUp().onTrue(s_climber.runOnce(() -> s_climber.setAngle(Constants.Climber.climbAngle)));       // POV Up - Set climber to climb angle
        driver.povDown().onTrue(s_climber.runOnce(() -> s_climber.setAngle(Constants.Climber.floorAngle)));     // POV Down - Set climber to down angle
        driver.rightBumper().onTrue(new FloorCoralIntake(s_carriage, s_elevator));                              // Right bumper - Coral Floor Intake
        
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
        driver.leftTrigger().whileTrue
        (
            new SequentialCommandGroup
            (
                new ParallelCommandGroup
                (
                    new PositionAlign(s_swerve, s_carriage, s_elevator, driver), // 1. Align to position
                    new ConditionalCommand  // Go to height if within X inches of target
                    (
                        // If the condition is TRUE, go to height
                        new GoToHeightPreset(s_carriage, s_elevator, s_swerve),  
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
        );

        /*  Add back in once SysID is completed
        // Start Button - Cancel All Commands
        driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        */        

        //////////////////////////////////////////////////////////////////////////////////////////
        /// OPERATOR CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        operator.button(Constants.Button.height.L1).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L1)));
        operator.button(Constants.Button.height.L2).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L2)));
        operator.button(Constants.Button.height.L3).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L3)));
        operator.button(Constants.Button.height.L4).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.L4)));
        operator.button(Constants.Button.height.Barge).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.Barge)));

        operator.button(Constants.Button.height.A1).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.A1)));
        operator.button(Constants.Button.height.A2).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.Elevator.HeightPresets.A2)));

        operator.button(Constants.Button.location.A).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.A)));
        operator.button(Constants.Button.location.B).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.B)));
        operator.button(Constants.Button.location.C).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.C)));
        operator.button(Constants.Button.location.D).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.D)));
        operator.button(Constants.Button.location.E).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.E)));
        operator.button(Constants.Button.location.F).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.F)));
        operator.button(Constants.Button.location.G).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.G)));
        operator.button(Constants.Button.location.H).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.H)));
        operator.button(Constants.Button.location.I).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.I)));
        operator.button(Constants.Button.location.J).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.J)));
        operator.button(Constants.Button.location.K).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.K)));
        operator.button(Constants.Button.location.L).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.L)));

        operator.button(Constants.Button.location.LeftCoral).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.LeftCoral)));
        operator.button(Constants.Button.location.RightCoral).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.RightCoral)));

        operator.button(Constants.Button.location.Barge).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.Barge)));
        operator.button(Constants.Button.location.Processor).onTrue(s_swerve.runOnce(() -> s_swerve.setDestination(Constants.Destinations.Processor)));

        operator.button(Constants.Button.location.H).onTrue(s_climber.runOnce(() -> s_climber.openHopper()));

        //////////////////////////////////////////////////////////////////////////////////////////
        // SYSID ROUTINES
        //////////////////////////////////////////////////////////////////////////////////////////

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(s_swerve.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(s_swerve.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(s_swerve.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(s_swerve.sysIdQuasistatic(Direction.kReverse));
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
    }

    public void registerNamedCommands()
    {
        NamedCommands.registerCommand
        (
            "GoToL1Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L1), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToL2Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L2), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToL3Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L3), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToL4Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.L4), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToBargeHeight",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.Barge), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToA1Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.A1), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToA2Height",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.A2), s_elevator)
        );

        NamedCommands.registerCommand
        (
            "GoToHandoffHeight",
            new RunCommand(() -> s_elevator.setHeight(Constants.Elevator.HeightPresets.handoffHeight), s_elevator)
        );

        NamedCommands.registerCommand   // TODO - check
        (
            "ScoreCoral",
            new AutoScore(s_carriage, s_elevator, false)
        );

        NamedCommands.registerCommand   // TODO - check
        (
            "ScoreAlgae",
            new AutoScore(s_carriage, s_elevator, true)
        );
    }
}
