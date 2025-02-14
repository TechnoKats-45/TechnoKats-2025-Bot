package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carriage extends SubsystemBase
{
    // CAN IDs:
    int coralMotorID = 1;           // TODO - Replace with actual CAN ID
    int coralCANdiID = 2;           // TODO - Replace with actual CAN ID
    int algaeMotorID = 3;           // TODO - Replace with actual CAN ID
    int algaeAngleMotorID = 4;      // TODO - Replace with actual CAN ID
    int algaeCANdiID = 5;           // TODO - Replace with actual CAN ID
    double algaeAngle;
    double algaeAnglePreset;

    private TalonFX coralMotor;
    private TalonFX algaeMotor;
    private TalonFX algaeAngleMotor;
    private CANdi coralCANdi;   // Coral Digital Sensor(s)
    private CANdi algaeCANdi;   // Algae Digital Sensor and Algae Angle Encoder

    // PID Declarations
    private final VelocityVoltage coral_velocity = new VelocityVoltage(0);
    private final VelocityVoltage algae_velocity = new VelocityVoltage(0);
    private final PositionTorqueCurrentFOC algae_angle = new PositionTorqueCurrentFOC(0);
    
    public Carriage()
    {
        // Initialize motors and sensors
        coralMotor = new TalonFX(coralMotorID);
        coralCANdi = new CANdi(coralCANdiID);

        algaeMotor = new TalonFX(algaeMotorID);
        algaeAngleMotor = new TalonFX(algaeAngleMotorID);
        algaeCANdi = new CANdi(algaeCANdiID);

        configCoralMotor();
        configAlgaeMotor();
        configAlgaeAngleMotor();
    }

    ///////////////////////////////////////////////////////////////////////////
    /// ALGAE Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public double getAlgaeAngle()
    {
        algaeAngle = algaeCANdi.getPWM1Position().getValueAsDouble();   // Get the angle of the algae (-16384.0 to 16383.999755859375)
        algaeAngle = ((algaeAngle % 360) + 360) % 360;  // Normalize the angle to the range [0, 360)
        return algaeAngle;
    }

    public void setAlgaeAngle(double angle)
    {
        algaeAngleMotor.setControl(algae_angle.withPosition(angle/360));
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
        algaeMotor.setControl(algae_velocity.withVelocity(speed));
    }

    ///////////////////////////////////////////////////////////////////////////
    /// CORAL Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public boolean isCoralDetected()
    {
        return coralCANdi.getS2State().getValueAsDouble() > 0;
    }

    public void setCoralSpeed(double speed)
    {
        coralMotor.setControl(coral_velocity.withVelocity(speed));
    }
    
    ///////////////////////////////////////////////////////////////////////////
    /// CONFIGS
    /////////////////////////////////////////////////////////////////////////// 
    
    public void configCoralMotor()
    {
        TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        algaeConfigs.Slot0.kS = 0;// TODO - Tune
        algaeConfigs.Slot0.kV = 0;// TODO - Tune
        algaeConfigs.Slot0.kP = 0;// TODO - Tune
        algaeConfigs.Slot0.kI = 0;// TODO - Tune
        algaeConfigs.Slot0.kD = 0;// TODO - Tune
        
        // Peak output of 8 volts
        algaeConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = algaeMotor.getConfigurator().apply(algaeConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public void configAlgaeMotor()
    {
        TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        algaeConfigs.Slot0.kS = 0;// TODO - Tune
        algaeConfigs.Slot0.kV = 0;// TODO - Tune
        algaeConfigs.Slot0.kP = 0;// TODO - Tune
        algaeConfigs.Slot0.kI = 0;// TODO - Tune
        algaeConfigs.Slot0.kD = 0;// TODO - Tune
        
        // Peak output of 8 volts
        algaeConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = algaeMotor.getConfigurator().apply(algaeConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public void configAlgaeAngleMotor()
    {
        TalonFXConfiguration algaeAngleMotorConfigs = new TalonFXConfiguration();
        algaeAngleMotorConfigs.Slot0.kP = 0;// TODO - Tune
        algaeAngleMotorConfigs.Slot0.kI = 0;// TODO - Tune
        algaeAngleMotorConfigs.Slot0.kD = 0;// TODO - Tune
        
        // Peak output of 120 A
        algaeAngleMotorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
            .withPeakReverseTorqueCurrent(Amps.of(-120));

            algaeAngleMotorConfigs.Feedback = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1)
                .withFeedbackRemoteSensorID(algaeCANdiID);

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = algaeAngleMotor.getConfigurator().apply(algaeAngleMotorConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    
        /* Make sure we start at 0 */
        algaeAngleMotor.setPosition(0);
    }

    public void printDiagnostics()
    {
        SmartDashboard.putBoolean("Algae Detecected", isAlgaeDetected());
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        SmartDashboard.putNumber("Algae Intake Angle", getAlgaeAngle());
    }

    // Custom SYS ID:
    public void determineKs()   // TODO - Assign to button
    {
        determineKsForMotor(coralMotor, "Coral Motor");
        determineKsForMotor(algaeMotor, "Algae Motor");
        determineKsForMotor(algaeAngleMotor, "Algae Angle Motor");
    }
    
    private void determineKsForMotor(TalonFX motor, String motorName)   // TODO - Assign to button
    {
        double voltage = 0.0;
        double velocity = 0.0;
    
        while (voltage <= 12.0) // Prevent over-volting
        { 
            motor.setVoltage(voltage);
            velocity = motor.getVelocity().getValueAsDouble(); // Assuming this method gets velocity
    
            SmartDashboard.putNumber(motorName + " Testing Voltage", voltage);
            SmartDashboard.putNumber(motorName + " Velocity", velocity);
    
            if (Math.abs(velocity) > 0.01)  // Detect motion (adjust threshold if needed)
            { 
                System.out.println(motorName + " motion detected at " + voltage + "V");
                break;
            }
    
            voltage += 0.1;
            try 
            {
                Thread.sleep(500); // Small delay to allow motor to react
            } 
            catch (InterruptedException e) 
            {
                Thread.currentThread().interrupt();
            }
        }
    
        motor.setVoltage(0); // Stop the motor
        System.out.println("Final kS estimate for " + motorName + ": " + voltage);
    }
    
}
