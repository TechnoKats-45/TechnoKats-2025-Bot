package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    //int algaeCANdiID = 5;
    double algaeAngle;
    double algaeAnglePreset;

    double currentAnglePreset;

    private TalonFX coralMotor;
    private TalonSRX algaeMotor;
    private TalonFX algaeAngleMotor;
    private CANdi coralCANdi;   // Coral Digital Sensor(s)

    // PID control on angle (in degrees)
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

    // PID Declarations
    private final VelocityVoltage coral_velocity = new VelocityVoltage(0);
    
    public Carriage()
    {
        // Initialize motors and sensors
        coralMotor = new TalonFX(coralMotorID);
        coralCANdi = new CANdi(coralCANdiID);

        algaeMotor = new TalonSRX(algaeMotorID);
        algaeAngleMotor = new TalonFX(algaeAngleMotorID);
        algaeAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        //algaeCANdi = new CANdi(algaeCANdiID);

        configCoralMotor();
        configAlgaeAngle();
    }

    ///////////////////////////////////////////////////////////////////////////
    /// ALGAE Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public double getAlgaeAngle()
    {
        return algaeAngleMotor.getPosition().getValueAsDouble();
    }

    public void setAlgaeAngle(double angle) 
    {
        algaeAngleMotor.setControl(motionMagicControl.withPosition(angle));
        SmartDashboard.putNumber("Desired Angle", angle);
    }

    public void setAlgaeAngle()
    {
        setAlgaeAngle(currentAnglePreset);
    }

    public void setAnglePreset(double passedPreset)
    {
        currentAnglePreset = passedPreset;
    }

    // Returns true if the current angle is within a specified tolerance of the preset angle.
    public boolean isAlgaeAngleAligned() 
    {
        return Math.abs(getAlgaeAngle() - currentAnglePreset) <= Constants.Carriage.algaeAngleTolerance;
    }

    /*
    public boolean isAlgaeDetected()
    {
        return algaeCANdi.getS2State().getValueAsDouble() > 0;
    }
    */

    public void setAlgaeSpeed(double speed)
    {
        algaeMotor.set(ControlMode.PercentOutput, speed);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// CORAL Helpers
    /////////////////////////////////////////////////////////////////////////// 

    public boolean isCoralDetected()
    {
        //SmartDashboard.putNumber("Coral RAW Data", coralCANdi.getS2State().getValueAsDouble());
        return (coralCANdi.getS2State().getValueAsDouble() < 2);    // For some reason false is 2, and true is 1... we're just rollin' with it.
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
    
    public void configAlgaeAngle()
    {
        TalonFXConfiguration algaeAngleConfigs = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();
    
        // Enable stator current limit.
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.StatorCurrentLimitEnable = true; // or true as needed

        FeedbackConfigs fdb = algaeAngleConfigs.Feedback;
        fdb.FeedbackRemoteSensorID = coralCANdiID;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
        fdb.SensorToMechanismRatio = 1;
        fdb.RotorToSensorRatio = 14.0/39.0;

        /* Configure Motion Magic parameters as needed */
        MotionMagicConfigs mm = algaeAngleConfigs.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));
            //.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));

        algaeAngleConfigs.Slot0.kS = 0;// TODO - Tune
        algaeAngleConfigs.Slot0.kV = 0;// TODO - Tune
        algaeAngleConfigs.Slot0.kP = 10;// TODO - Tune  // 25 w/ elastic    // was 20
        algaeAngleConfigs.Slot0.kI = 0;// TODO - Tune
        algaeAngleConfigs.Slot0.kD = 1;// TODO - Tune

        algaeAngleConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Peak output of 8 volts
        algaeAngleConfigs.Voltage.withPeakForwardVoltage(Volts.of(16))
            .withPeakReverseVoltage(Volts.of(-16));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = algaeAngleMotor.getConfigurator().apply(algaeAngleConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

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
        //SmartDashboard.putBoolean("Algae Detecected", isAlgaeDetected());
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        SmartDashboard.putNumber("AlgaeAngle", getAlgaeAngle());
    }    
}
