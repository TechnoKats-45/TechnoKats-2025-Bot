package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.core.CoreCANdi;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class Climber extends SubsystemBase
{
    // CAN IDs:
    int winchMotorID = 6;           // TODO - Replace with actual CAN ID
    double climberAngle;
    double minimumInAngle = 600;
    double maximumOutAngle = 310;
    private double lastAngle = 0.0;
    private int rotationCount = 0;


    private TalonFX winchMotor;
    private boolean climbEnabled = false;
    DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    //private final CANdi CANdi = new CANdi(Constants.Elevator.elevatorCANdiID);

    public Climber()
    {
        // Initialize motors and sensors
        winchMotor = new TalonFX(winchMotorID);
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        //configWinchMotor();   // Breaks things
                
        // Configure the CANdi for basic use
        //CANdiConfiguration configs = new CANdiConfiguration();

        // Write these configs to the CANdi
        //CANdi.getConfigurator().apply(configs);
        
    }
    
    public double getAngle()
    {
        double newAngle = encoder.get() * 360;  // Convert from [0,1] to degrees
    
        // Detect wrap-around
        if (lastAngle > 300 && newAngle < 60) {  // Wrap from 360 to 0
            rotationCount++;
        } else if (lastAngle < 60 && newAngle > 300) {  // Wrap from 0 to 360
            rotationCount--;
        }
    
        lastAngle = newAngle;  // Store current angle for next comparison
    
        return (rotationCount * 360) + newAngle;  // Return continuous position
    }
    

    public void enableClimb()
    {
        climbEnabled = true;
    }

    public void disableClimb()
    {
        climbEnabled = false;
    }

    public boolean isClimbEnabled()
    {
        return climbEnabled;
    }

    public void ManualClimber(CommandXboxController controller, CommandXboxController controller2)
    {
        double angle = getAngle();  // Get current climber position
    
        if (controller2.povDown().getAsBoolean() && climbEnabled && getAngle() < 235)   // Moving IN (Stowing)
        {  
            winchMotor.set(-1);
        } 
        else if (controller2.povUp().getAsBoolean() && climbEnabled && getAngle() > -140)  // Moving OUT (Extending)
        {  
            winchMotor.set(1);
        } 
        else 
        {
            winchMotor.set(0);  // Stop if out of bounds or no input
        }
    }
    

    public void printDiagnostics()
    {
        SmartDashboard.putNumber("Climber Encoder Angle", getAngle());
        SmartDashboard.putNumber("Winch Current", winchMotor.getSupplyCurrent().getValueAsDouble());
    }    
}
