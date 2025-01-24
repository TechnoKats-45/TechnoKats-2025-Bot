package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Climber
{
    // CAN IDs:
    int winchMotorID = 6;           // TODO - Replace with actual CAN ID
    int EncoderPort = 0;            // TODO - Replace with actual DIO port number on RIO

    private TalonFX winchMotor;
    private DutyCycleEncoder m_absoluteEncoder;

    // PID Declarations
    private final PositionTorqueCurrentFOC winch_angle = new PositionTorqueCurrentFOC(0);
    // TODO - PID Stuff

    public Climber()
    {
        // Initialize motors and sensors
        winchMotor = new TalonFX(winchMotorID);
        m_absoluteEncoder = new DutyCycleEncoder(EncoderPort);


        configWinchMotor();
    }

    public void setAngle(double angle)
    {
        winchMotor.setControl(winch_angle.withPosition(angle/360));
    }
    
    public double getAngle()
    {
        return m_absoluteEncoder.get() * 360;
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
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemotePWM0) // Use PWM0 port on RoboRIO         // TODO - I don't think I can do closed loop unless I get another CANdi


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
}
