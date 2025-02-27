package frc.robot.subsystems;

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
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class Climber extends SubsystemBase
{
    // CAN IDs:
    int winchMotorID = 6;           // TODO - Replace with actual CAN ID
    double climberAngle;
    double minimumInAngle = 600;
    double maximumOutAngle = 310;

    private TalonFX winchMotor;
    private boolean climbEnabled = false;
    private final CANdi candi = new CANdi(Constants.Elevator.elevatorCANdiID);

    public Climber()
    {
        // Initialize motors and sensors
        winchMotor = new TalonFX(winchMotorID);
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        //configWinchMotor();   // Breaks things
    }
    
    public double getAngle()
    {
        //return winchMotor.getPosition().getValueAsDouble();
        var candiPos2 = candi.getPWM2Position();

        return candiPos2.getValue().in(Degrees);
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

    public void ManualClimber(CommandXboxController controller, CommandXboxController controller2)  // Set max 3.95 degree as max climb // 0.78 minimum angle (STOWED)
    {
        if (controller2.povUp().getAsBoolean() && climbEnabled && (Math.abs(getAngle()) > 300))   // OUT of BOT   minimumInAngle                 //-600 is all the way IN, -300 is all the way out
        {  // Down
            winchMotor.set(-1);
        } else if (controller2.povDown().getAsBoolean() && climbEnabled & (Math.abs(getAngle()) < 600))  // IN TO BOT maximumOutAngllimn
        { // Up
            winchMotor.set(1);
        } else {
            winchMotor.set(0);
        }
    }

    public void printDiagnostics()
    {
        SmartDashboard.putNumber("Climber Encoder Angle", getAngle());
        SmartDashboard.putNumber("Winch Current", winchMotor.getSupplyCurrent().getValueAsDouble());
    }    
}
