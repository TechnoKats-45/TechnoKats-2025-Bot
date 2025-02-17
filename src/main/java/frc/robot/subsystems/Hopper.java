package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Hopper extends SubsystemBase
{
    private Servo m_hatchServo;

    public Hopper()
    {
        // Initialize motors and sensors
    }

    public boolean isCoralDetected()
    {
        // TODO - Implement
        return false;
    }

    public void openHopper()
    {
        m_hatchServo.setAngle(Constants.Climber.hopperReleaseAngle);
    }

    public void printDiagnostics()
    {

    }
    
}
