package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;


public class Elevator extends SubsystemBase
{
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;
    private Follower follower;
    
    double elevatorAngle;
    double elevatorHeight;
    double currentHeightPreset;

    private CANdi elevatorCANdi;

    // PID Declarations
    private final PositionTorqueCurrentFOC elevator_angle = new PositionTorqueCurrentFOC(0);

    public Elevator()
    {
        // Initialize motors and sensors
        elevatorMotor1 = new TalonFX(Constants.Elevator.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        //elevatorMotor2 = new TalonFX(Constants.Elevator.elevatorMotor2ID);
        //elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        follower = new Follower(Constants.Elevator.elevatorMotor1ID, false);
        elevatorCANdi = new CANdi(Constants.Elevator.elevatorCANdiID);

        // How to control it:
            //elevatorMotor1.set(x);
            //elevatorMotor2.setControl(follower);

        configElevator();
    }

    public boolean isAligned()
    {
        return (Math.abs(getHeight() - currentHeightPreset)) <= (Constants.Elevator.elevatorHeightTolerance);
    }

    public void setHeightPreset(double angle)
    {
        currentHeightPreset = angle;    // TODO - Change to height, rn it's angle
    }

    public double getHeightPreset()
    {
        return currentHeightPreset;
    }

    public void setAngle(double angle) // Set to angle
    {
        elevatorMotor1.setControl(elevator_angle.withPosition(angle/360));
        //elevatorMotor2.setControl(follower);
    }

    public void setAngle()  // Set to currentHeightPreset   // TODO - Change to height, rn it's angle
    {
        elevatorMotor1.setControl(elevator_angle.withPosition(currentHeightPreset/360));
        //elevatorMotor2.setControl(follower);
    }

    public double getAngle()
    {
        elevatorAngle = elevatorCANdi.getPWM1Position().getValueAsDouble();   // Get the angle of the elevator (-16384.0 to 16383.999755859375)
        elevatorAngle = ((elevatorAngle % 360) + 360) % 360;  // Normalize the angle to the range [0, 360)
        return elevatorAngle;
    }

    public void setHeight(double setPoint)
    {
        // TODO - Implement
    }

    public double getHeight()
    {
        elevatorHeight = getAngle() * Constants.Elevator.GearRatio; // TODO - Implement
        return 0;
    }

    public void ManualElevator(CommandXboxController controller)
    {
        if(controller.getRightY() < -Constants.STICK_DEADBAND)  // Down
        {
            elevatorMotor1.set(.2);  // Go up
        }
        else if(controller.getRightY() > Constants.STICK_DEADBAND) // Up
        {
            elevatorMotor1.set(-.2); // Go down
        }
        else
        {
            // Hold Position
            elevatorMotor1.set(0);
        }
    }

    public void configElevator()
    {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 40;
        limitConfigs.StatorCurrentLimitEnable = true;

        /* Configure gear ratio */
        FeedbackConfigs fdb = elevatorConfig.Feedback;
        fdb.SensorToMechanismRatio = 10; // 10 rotor rotations per mechanism rotation       // TODO - Tune

        /* Configure Motion Magic */
        MotionMagicConfigs mm = elevatorConfig.MotionMagic; // TODO - TUNE
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));  // Take approximately 0.1 seconds to reach max accel 
        
        elevatorConfig.Slot0.kS = 0;        // TODO - Tune
        elevatorConfig.Slot0.kG = 0.09;     // Estimated from ReCalc        // TODO - Tune
        elevatorConfig.Slot0.kV = 3.11;     // Estimated from ReCalc        // TODO - Tune
        elevatorConfig.Slot0.kA = 0.01;     // Estimated from ReCalc        // TODO - Tune
        elevatorConfig.Slot0.kP = 0;        // TODO - Tune
        elevatorConfig.Slot0.kI = 0;        // TODO - Tune
        elevatorConfig.Slot0.kD = 0;        // TODO - Tune

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = elevatorMotor1.getConfigurator().apply(elevatorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void printDiagnostics()
    {
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putNumber("Elevator Angle", getAngle());
        SmartDashboard.putNumber("currentHeightPreset", currentHeightPreset);
        SmartDashboard.putBoolean("Elevator Aligned", isAligned());
    }

    // SYS ID Stuff (Manual)
    public void determineKs()   // TODO - Assign to button
    {
        double voltage = 0.0;
        double velocity = 0.0;
        
        while (voltage <= 12.0)     // Prevent over-volting
        {
            elevatorMotor1.setVoltage(voltage);
            velocity = elevatorMotor1.getVelocity().getValueAsDouble(); // Assuming this method gets velocity
    
            SmartDashboard.putNumber("Testing Voltage", voltage);
            SmartDashboard.putNumber("Elevator Velocity", velocity);
    
            if (Math.abs(velocity) > 0.01)  // Detect motion (adjust threshold if needed)
            { 
                System.out.println("Motion detected at " + voltage + "V");
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
    
        elevatorMotor1.setVoltage(0); // Stop the motor
        System.out.println("Final kS estimate: " + voltage);
    }
}
