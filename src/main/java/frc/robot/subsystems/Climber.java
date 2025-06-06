package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
        if (lastAngle > 300 && newAngle < 60) 
        {  // Wrap from 360 to 0
            rotationCount++;
        } 
        else if (lastAngle < 60 && newAngle > 300) 
        {  // Wrap from 0 to 360
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

    public boolean isClimberAligned()
    {
        double angle = getAngle();
        return ((angle < Constants.Climber.climbAngle + 15) && (angle > Constants.Climber.climbAngle - 15));  // Check if within 1 degree of target angle
    }

    public void setClimberSpeed(double speed)
    {
        winchMotor.set(speed);
    }

    public void ManualClimber(CommandXboxController controller, CommandXboxController controller2)
    {
        double angle = getAngle();  // Get current climber position

        if ((controller2.povDown().getAsBoolean() || controller2.povDownLeft().getAsBoolean() || controller2.povDownRight().getAsBoolean()) && climbEnabled)   // Moving IN (Stowing)  // 300
        {  
            winchMotor.set(-1);
        } 
        else if ((controller2.povUp().getAsBoolean() || controller2.povUpRight().getAsBoolean() || controller2.povUpLeft().getAsBoolean()) && climbEnabled)  // Moving OUT (Extending)        // =-140
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
