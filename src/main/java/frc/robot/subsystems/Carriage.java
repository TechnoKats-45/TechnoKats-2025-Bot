package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Carriage extends SubsystemBase
{
    // CAN IDs:
    int coralMotorID = 1;
    int coralCANdiID = 2;
    int algaeMotorID = 3;
    int algaeAngleMotorID = 4;
    int algaeCANdiID = 5;
    double algaeAngle;
    double algaeAnglePreset;

    private TalonFX coralMotor;
    private TalonFX algaeMotor;
    private TalonSRX algaeAngleMotor;
    private CANdi coralCANdi;   // Coral Digital Sensor(s)
    private CANdi algaeCANdi;   // Algae Digital Sensor and Algae Angle Encoder

    // PID Declarations
    private final VelocityVoltage coral_velocity = new VelocityVoltage(0);
    
    public Carriage()
    {
        // Initialize motors and sensors
        coralMotor = new TalonFX(coralMotorID);
        coralCANdi = new CANdi(coralCANdiID);

        algaeMotor = new TalonFX(algaeMotorID);
        algaeAngleMotor = new TalonSRX(algaeAngleMotorID);

        algaeCANdi = new CANdi(algaeCANdiID);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// ALGAE Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public double getAlgaeAngle()
    {
        algaeAngle = coralCANdi.getPWM1Position().getValueAsDouble();

        return algaeAngle;
    }
    
    public void ManualAlgaeAngle(CommandXboxController controller) 
    {
        if (controller.povLeft().getAsBoolean()) {  // Down
            algaeAngleMotor.set(ControlMode.PercentOutput, .25);
        } 
        else if (controller.povRight().getAsBoolean())   // Up
        { // Up
            algaeAngleMotor.set(ControlMode.PercentOutput, -.5);
        } 
        else 
        {
            algaeAngleMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setAlgaeAngle(double angle)
    {
            // TODO - Remove this
    }

    public boolean isAlgaeAngleAligned()
    {
        return (Math.abs(getAlgaeAngle() - algaeAnglePreset)) <= (Constants.Carriage.algaeAngleTolerance);
    }

    public boolean isAlgaeDetected()
    {
        return algaeCANdi.getS2State().getValueAsDouble() > 0;
    }

    public void setAlgaeSpeed(double speed)
    {
        //algaeMotor.setControl(algae_velocity.withVelocity(speed));
    }

    ///////////////////////////////////////////////////////////////////////////
    /// CORAL Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public boolean isCoralDetected()
    {
        //SmartDashboard.putNumber("Coral RAW Data", coralCANdi.getS2State().getValueAsDouble());
        return (coralCANdi.getS2State().getValueAsDouble() < 2);    // For some reason fasle is 2, and true is 1... we're just rollin' with it.
    }

    public void setCoralSpeed(double speed)
    {
        coralMotor.setControl(coral_velocity.withVelocity(speed));
        //SmartDashboard.putNumber("Coral Speed", speed);
    }

    public void setCoralSpeed(double speed, Elevator s_elevator)
    {
        if (s_elevator.getAnglePreset() == Constants.Elevator.AnglePresets.L4)
        {
            speed = Constants.Carriage.autoCoralScoreSpeed;
        }
        coralMotor.setControl(coral_velocity.withVelocity(speed));
        //SmartDashboard.putNumber("Coral Speed", speed);
    }

    public void setCoralSpeedDumb(double dumbSpeed)
    {
        coralMotor.set(dumbSpeed);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    /// CONFIGS
    /////////////////////////////////////////////////////////////////////////// 
    
    public void configCoralMotor()
    {
        TalonFXConfiguration coralConfigs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        coralConfigs.Slot0.kS = 0;// TODO - Tune
        coralConfigs.Slot0.kV = 0;// TODO - Tune
        coralConfigs.Slot0.kP = .1;// TODO - Tune
        coralConfigs.Slot0.kI = 0;// TODO - Tune
        coralConfigs.Slot0.kD = 0;// TODO - Tune
        
        // Peak output of 8 volts
        coralConfigs.Voltage.withPeakForwardVoltage(Volts.of(16))
            .withPeakReverseVoltage(Volts.of(-16));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = coralMotor.getConfigurator().apply(coralConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public void printDiagnostics()
    {
        SmartDashboard.putBoolean("Algae Detecected", isAlgaeDetected());
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        SmartDashboard.putNumber("Algae Intake Angle", getAlgaeAngle());
    }    
}
