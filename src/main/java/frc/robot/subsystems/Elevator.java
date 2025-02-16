package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.core.CoreCANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Elevator extends SubsystemBase 
{
    private TalonFX elevatorMotor1;
    
    // This preset is now in degrees.
    double currentHeightPreset;

    // Using CANdi for diagnostics (optional)
    private CoreCANdi elevatorCANdi;

    // PID control on angle (in degrees)
    private final PositionTorqueCurrentFOC elevator_angle = new PositionTorqueCurrentFOC(0);

    public Elevator() {
        // Initialize motors and sensors
        elevatorMotor1 = new TalonFX(Constants.Elevator.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorCANdi = new CANdi(Constants.Elevator.elevatorCANdiID);

        configElevator();
    }
    
    /**
     * Returns true if the current angle is within a specified tolerance of the preset angle.
     */
    public boolean isAligned() {
        return Math.abs(getAngle() - currentHeightPreset) <= Constants.Elevator.elevatorHeightTolerance;
    }

    public void setHeightPreset(double angle) {
        currentHeightPreset = angle;
    }

    public double getHeightPreset() {
        return currentHeightPreset;
    }

    /**
     * Command the elevator to a specific angle (in degrees).
     */
    public void setAngle(double angle) 
    {
        // Convert desired physical angle to sensor space by adding the sensor offset.
        elevatorMotor1.setControl(elevator_angle.withPosition(angle));
        SmartDashboard.putNumber("Desired Angle", angle);
    }
    
    /**
     * Overloaded method that uses the current preset angle.
     */
    public void setAngle() 
    {
        setAngle(currentHeightPreset);
    }

    /**
     * Returns the measured angle from the CANdi sensor (for diagnostics).
     */
    public double getAngle() 
    {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    public void ManualElevator(CommandXboxController controller) 
    {
        if (controller.getRightY() < -Constants.STICK_DEADBAND) {  // Down
            elevatorMotor1.set(0.2);
        } else if (controller.getRightY() > Constants.STICK_DEADBAND) { // Up
            elevatorMotor1.set(-0.2);
        } else {
            elevatorMotor1.set(0);
        }
    }

    public void configElevator() 
    {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();
    
        // Enable stator current limit.
        limitConfigs.StatorCurrentLimit = 40;
        limitConfigs.StatorCurrentLimitEnable = false; // or true as needed
    
        /* Configure the Talon FX to use the CANdi sensor as its feedback source.
           Using the helper method withRemoteCANdiPwm1, we pass in the CANdi object.
           Then, we set the SensorToMechanismRatio to convert the remote sensor's
           native units (assumed to be rotations) into degrees. For example, if one
           mechanism rotation equals 360° and the CANdi outputs in encoder rotations,
           and your mechanism's reduction is given by your constants, you can set:
           360.0 / Constants.Elevator.MECHANISM_TO_ENCODER.
        */
        FeedbackConfigs fdb = elevatorConfig.Feedback;
        fdb.FeedbackRemoteSensorID = Constants.Elevator.elevatorCANdiID;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        fdb.SensorToMechanismRatio = 1/Constants.Elevator.MECHANISM_TO_ENCODER;
        fdb.RotorToSensorRatio = 11.9*12;

        /* Configure Motion Magic parameters as needed */
        /*
        MotionMagicConfigs mm = elevatorConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        */

        // PID and feedforward tuning constants
        elevatorConfig.Slot0.kS = 0.9;   // Tune as needed. / was 0.9
        elevatorConfig.Slot0.kG = 10;   // Tune as needed.    // was .09
        elevatorConfig.Slot0.kV = 3.11;   // Tune as needed.    // was 3.11
        elevatorConfig.Slot0.kA = 0.01;   // Tune as needed.    // was 0.01
        elevatorConfig.Slot0.kP = 50;      // Your tuned P value.    // was 5
        elevatorConfig.Slot0.kI = 0;      // Tune as needed.
        elevatorConfig.Slot0.kD = 0;      // Tune as needed.
        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevatorMotor1.getConfigurator().apply(elevatorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }
    
    public void printDiagnostics() 
    {
        SmartDashboard.putNumber("Current Preset Angle", currentHeightPreset);
        SmartDashboard.putBoolean("Elevator Aligned", isAligned());
        SmartDashboard.putNumber("CANDdi from TALON Angle", elevatorMotor1.getPosition().getValueAsDouble());
    }

    // SYS ID Stuff (Manual)
    public void determineKs() {   // TODO - Assign to button
        double voltage = 0.0;
        double velocity = 0.0;
        
        while (voltage <= 12.0) {     // Prevent over-volting
            elevatorMotor1.setVoltage(voltage);
            velocity = elevatorMotor1.getVelocity().getValueAsDouble();
    
            SmartDashboard.putNumber("Testing Voltage", voltage);
            SmartDashboard.putNumber("Elevator Velocity", velocity);
    
            if (Math.abs(velocity) > 0.01) {
                System.out.println("Motion detected at " + voltage + "V");
                break;
            }
    
            voltage += 0.1;
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    
        elevatorMotor1.setVoltage(0);
        System.out.println("Final kS estimate: " + voltage);
    }
}
