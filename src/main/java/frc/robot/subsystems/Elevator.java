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
import com.ctre.phoenix6.controls.MotionMagicVoltage;

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
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);


    public Elevator() 
    {
        // Initialize motors and sensors
        elevatorMotor1 = new TalonFX(Constants.Elevator.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorCANdi = new CANdi(Constants.Elevator.elevatorCANdiID);

        configElevator();
    }
    
    /**
     * Returns true if the current angle is within a specified tolerance of the preset angle.
    */
    public boolean isAligned() 
    {
        //return Math.abs(getHeight() - currentHeightPreset) <= Constants.Elevator.elevatorHeightTolerance;
        return false;
    }

    private double getHeightFromAngle(double angle) 
    {
        double minAngle = 8.01;
        double maxAngle = 19.0;
        double minHeight = 8.0;
        double maxHeight = 79.0;
        double height = minHeight + (angle - minAngle) * (maxHeight - minHeight) / (maxAngle - minAngle);
        return height;
    }

    private double getAngleFromHeight(double height) 
    {
        double minAngle = 8.01;
        double maxAngle = 19.0;
        double minHeight = 8.0;
        double maxHeight = 79.0;
        double angle = minAngle + (height - minHeight) * (maxAngle - minAngle) / (maxHeight - minHeight);
        return angle;
    }

    public double getHeight()
    {
        return(getHeightFromAngle(getAngle()));
    }
    
    public void setHeight(double height)
    {
        setAngle(getAngleFromHeight(height));
        SmartDashboard.putNumber("Desired Height", height);
    }

    public void GoToPreset()
    {
        setHeight(currentHeightPreset);
    }

    public void setHeightPreset(double heightPreset) 
    {
        currentHeightPreset = heightPreset;
    }

    public double getHeightPreset() 
    {
        return currentHeightPreset;
    }

    /**
     * Command the elevator to a specific angle (in degrees).
     */
    public void setAngle(double angle) 
    {
        // Convert desired physical angle to sensor space by adding the sensor offset.
        elevatorMotor1.setControl(motionMagicControl.withPosition(angle));
        SmartDashboard.putNumber("Desired Angle", angle);   // TODO - comment out
    }

    public void setAngle()  // TODO - remove
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
        if (controller.getLeftY() < -Constants.STICK_DEADBAND) {  // Down
            elevatorMotor1.set(0.2);
        } else if (controller.getLeftY() > Constants.STICK_DEADBAND) { // Up
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
        
        MotionMagicConfigs mm = elevatorConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(100))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        // PID and feedforward tuning constants
        elevatorConfig.Slot0.kS = 0.9;      // Tune as needed.
        elevatorConfig.Slot0.kG = 14;       // Tune as needed.
        elevatorConfig.Slot0.kV = 3.11;     // Tune as needed.
        elevatorConfig.Slot0.kA = 0.01;     // Tune as needed.
        elevatorConfig.Slot0.kP = 40;       // Tune as needed   // In an Elevator: If the elevator is far from the target position, the motor applies more power to get there quickly. However, it may not eliminate steady-state error, meaning the elevator might stop just short of the target.
        elevatorConfig.Slot0.kI = 1;// Was 5 // Tune as needed  // In an Elevator: If friction or gravity causes the elevator to stop just short of the target, the integral term will gradually increase power to eliminate this offset.
        elevatorConfig.Slot0.kD = 0;        // Tune as needed   // In an Elevator: If the elevator is moving too fast toward the target, the D term applies a braking effect, slowing it down before overshooting.
        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /* Retry config apply up to 5 times, report if failure */
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
        SmartDashboard.putNumber("Stashed Value", getHeightPreset());
        SmartDashboard.putBoolean("Elevator Aligned", isAligned());
        SmartDashboard.putNumber("CANDdi from TALON Angle", elevatorMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Height", getHeight());
        SmartDashboard.putNumber("Elevator Current", elevatorMotor1.getSupplyCurrent().getValueAsDouble());
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
