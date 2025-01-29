 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

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
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    //private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick operator = new CommandJoystick(1);

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

        //////////////////////////////////////////////////////////////////////////////////////////
        /// DRIVER CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////

        driver.a().whileTrue(s_swerve.applyRequest(() -> brake));                           // A button - brake the drivetrain
        driver.b().onTrue(s_swerve.runOnce(() -> s_swerve.seedFieldCentric()));             // B button - Reset the field-centric heading on B button press
        driver.leftTrigger().whileTrue(new autoAlign(s_swerve, s_carriage, s_elevator));    // Left trigger - Auto-align the robot w/ Operator selected location
        driver.povUp().onTrue(s_climber.runOnce(() -> s_climber.setAngle(Constants.Climber.climbAngle)));       // POV Up - Set climber to climb angle
        driver.povDown().onTrue(s_climber.runOnce(() -> s_climber.setAngle(Constants.Climber.floorAngle)));   // POV Down - Set climber to down angle

        /*  Add back in once SysID is completed
        // Start Button - Cancel All Commands
        driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        */        

        //////////////////////////////////////////////////////////////////////////////////////////
        /// OPERATOR CONTROLS
        //////////////////////////////////////////////////////////////////////////////////////////
        
        operator.button(Constants.Button.height.L1).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.L1)));
        operator.button(Constants.Button.height.L2).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.L2)));
        operator.button(Constants.Button.height.L3).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.L3)));
        operator.button(Constants.Button.height.L4).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.L4)));
        operator.button(Constants.Button.height.Barge).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.Barge)));

        operator.button(Constants.Button.height.A1).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.A1)));
        operator.button(Constants.Button.height.A2).onTrue(s_elevator.runOnce(() -> s_elevator.setHeightPreset(Constants.HeightPresets.A2)));

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

    public void registerNamedCommands()
    {
        // TODO
    }

    public void registerEventTriggers()
    {
        // TODO
        //Example:
            // Use event markers as triggers
            new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    }
}
