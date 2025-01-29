package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{
    // CAN IDs:
    int elevatorMotor1ID = 7;
    int elevatorMotor2ID = 8;
    int elevatorCANdiID = 9;

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
        elevatorMotor1 = new TalonFX(elevatorMotor1ID);
        elevatorMotor2 = new TalonFX(elevatorMotor2ID);
        follower = new Follower(elevatorMotor1ID, false);

        // How to control it:
            //elevatorMotor1.set(x);
            //elevatorMotor2.setControl(follower);

        configElevator();
    }

    public void setHeightPreset(double angle)
    {
        currentHeightPreset = angle;
    }

    public void setAngle(double angle)
    {
        elevatorMotor1.setControl(elevator_angle.withPosition(angle/360));
        elevatorMotor2.setControl(follower);
    }

    public double getAngle()
    {
        elevatorAngle = elevatorCANdi.getPWM1Position().getValueAsDouble();   // Get the angle of the algae (-16384.0 to 16383.999755859375)
        elevatorAngle = ((elevatorAngle % 360) + 360) % 360;  // Normalize the angle to the range [0, 360)
        return elevatorAngle;
    }

    public void setHeight()
    {
        // TODO - Implement
    }

    public double getHeight()
    {
        // TODO - Implement
        return 0;
    }

    public void configElevator()
    {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.Slot0.kP = 0; // An error of 1 rotation results in 60 A output       // TODO - Tune
        elevatorConfig.Slot0.kI = 0; // No output for integrated error                      // TODO - Tune
        elevatorConfig.Slot0.kD = 0; // A velocity of 1 rps results in 6 A output           // TODO - Tune
    }
}
