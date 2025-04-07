// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.AddressableLED;


public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;
  private Swerve s_swerve;
  private LEDSubsystem m_led;

  private final RobotContainer m_robotContainer;

  public Robot() 
  {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    m_robotContainer.printDiagnostics();
    m_robotContainer.updateLEDs();
  }
  
  @Override
  public void robotInit() 
  {
    PathfindingCommand.warmupCommand().schedule();
    m_robotContainer.s_elevator.setAngle(Constants.Elevator.AnglePresets.handoffAngle);

    //s_led.runPattern(LEDPattern.solid(Color.kGreen)).schedule();
    
    //s_swerve.seedFieldCentric();  // Need to replace with a value that is set on start of auto
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.s_elevator.setAngle(Constants.Elevator.AnglePresets.A2);
    m_robotContainer.s_elevator.setAnglePreset(Constants.Elevator.AnglePresets.A2);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}