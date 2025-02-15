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

public class Climber extends SubsystemBase
{
    // CAN IDs:
    int winchMotorID = 6;           // TODO - Replace with actual CAN ID
    double climberAngle;

    private TalonFX winchMotor;
    private DutyCycleEncoder m_absoluteEncoder;
    private Servo m_hatchServo;
    private CANdi climberCaNdi;

    // PID Declarations
    private final PositionTorqueCurrentFOC winch_angle = new PositionTorqueCurrentFOC(0);
    // TODO - PID Stuff

    public Climber()
    {
        // Initialize motors and sensors
        winchMotor = new TalonFX(winchMotorID);
        climberCaNdi = new CANdi(Constants.Elevator.elevatorCANdiID);

        //configWinchMotor();
    }

    public void setAngle(double angle)
    {
        winchMotor.setControl(winch_angle.withPosition(angle/360));
    }
    
    public double getAngle()
    {
        climberAngle = climberCaNdi.getPWM2Position().getValueAsDouble();   // Get the angle of the algae (-16384.0 to 16383.999755859375)
        climberAngle = ((climberAngle % 360) + 360) % 360;  // Normalize the angle to the range [0, 360)
        return climberAngle;
    }
    
    public void openHopper()
    {
        m_hatchServo.setAngle(Constants.Climber.hopperReleaseAngle);
    }

    public void configWinchMotor()
    {
        TalonFXConfiguration winchMotorConfigs = new TalonFXConfiguration();
            winchMotorConfigs.Slot0.kS = 0; // To account for friction, add 0.1 V of static feedforward                                                     // TODO - Tune
            winchMotorConfigs.Slot0.kV = 0; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second     // TODO - Tune
            winchMotorConfigs.Slot1.kP = 0; // An error of 1 rotation results in 60 A output                                                                // TODO - Tune
            winchMotorConfigs.Slot1.kI = 0; // No output for integrated error                                                                               // TODO - Tune
            winchMotorConfigs.Slot1.kD = 0; // A velocity of 1 rps results in 6 A output                                                                    // TODO - Tune

        // Peak output of 120 A
        winchMotorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
            .withPeakReverseTorqueCurrent(Amps.of(-120));

        winchMotorConfigs.Feedback = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM2)
            .withFeedbackRemoteSensorID(Constants.Elevator.elevatorCANdiID);


        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) 
        {
            status = winchMotor.getConfigurator().apply(winchMotorConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
            
        /* Make sure we start at 0 */
        winchMotor.setPosition(0);
    }

        public void printDiagnostics()
    {
        SmartDashboard.putNumber("Climber Winch Angle", getAngle());
    }

    // Custom SYS ID:
    public void determineKs() // TODO - Assign to button
    {
        double voltage = 0.0;
        double velocity = 0.0;
    
        while (voltage <= 12.0) // Prevent over-volting
        { 
            winchMotor.setVoltage(voltage);
            velocity = winchMotor.getVelocity().getValueAsDouble(); // Assuming this method gets velocity
    
            SmartDashboard.putNumber("Climber Testing Voltage", voltage);
            SmartDashboard.putNumber("Climber Winch Velocity", velocity);
    
            if (Math.abs(velocity) > 0.01)  // Detect motion (adjust threshold if needed)
            {
                System.out.println("Climber motion detected at " + voltage + "V");
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
    
        winchMotor.setVoltage(0); // Stop the motor
        System.out.println("Final kS estimate for Climber: " + voltage);
    }
    
}
