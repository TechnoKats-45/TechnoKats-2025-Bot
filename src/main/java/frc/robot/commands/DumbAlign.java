package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class DumbAlign extends Command 
{
    private final Limelight s_Limelight;
    private final Swerve s_swerve;

    private PIDController rotController;
    private PIDController strafeController;
    private PIDController forwardController;

    private enum Phase { ALIGN1, FORWARD, ALIGN2 }
    private Phase currentPhase = Phase.ALIGN1;

    private double startDistance = 0.0;
    private final double targetDistance = 1.0; // meters to drive forward
    private boolean finished = false;

    public DumbAlign(Limelight s_Limelight, Swerve s_swerve) 
    {
        this.s_Limelight = s_Limelight;
        this.s_swerve = s_swerve;

        rotController = new PIDController(0.02, 0, 0); // Tune these values
        strafeController = new PIDController(0.02, 0, 0);
        forwardController = new PIDController(0.5, 0, 0); // Forward drive tuning

        addRequirements(s_Limelight, s_swerve);
    }

    @Override
    public void initialize() 
    {
        currentPhase = Phase.ALIGN1;
        finished = false;
    }

    @Override
    public void execute() 
    {
        if (!s_Limelight.tagExists())
        {
            // No AprilTag — halt
            var request = new SwerveRequest.RobotCentric()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0);
            s_swerve.setControl(request);
            return;
        }

        switch (currentPhase) 
        {
            case ALIGN1:
                double yawError = s_Limelight.getYaw();      // degrees
                double strafeError = s_Limelight.getRX();    // assumed meters or degrees

                double yawOutput = rotController.calculate(yawError, 0);
                double strafeOutput = strafeController.calculate(strafeError, 0);

                var alignRequest = new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(strafeOutput)
                    .withRotationalRate(Math.toRadians(yawOutput)); // Convert to radians/sec
                s_swerve.setControl(alignRequest);

                if (Math.abs(yawError) < 1.0 && Math.abs(strafeError) < 0.05) 
                {
                    startDistance = s_swerve.getState().Pose.getTranslation().getY();
                    currentPhase = Phase.FORWARD;
                }
                break;

            case FORWARD:
                double currentY = s_swerve.getState().Pose.getTranslation().getY();
                double drivenDistance = Math.abs(currentY - startDistance);

                double forwardOutput = forwardController.calculate(drivenDistance, targetDistance);

                var forwardRequest = new SwerveRequest.RobotCentric()
                    .withVelocityX(forwardOutput)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0);
                s_swerve.setControl(forwardRequest);

                if (Math.abs(drivenDistance - targetDistance) < 0.05) 
                {
                    currentPhase = Phase.ALIGN2;
                }
                break;

            case ALIGN2:
                double xError = s_Limelight.getRX(); // Horizontal
                double zError = s_Limelight.getRY(); // Forward/backward

                double finalStrafe = strafeController.calculate(xError, 0);
                double finalForward = forwardController.calculate(zError, 0);

                var finalRequest = new SwerveRequest.RobotCentric()
                    .withVelocityX(finalForward)
                    .withVelocityY(finalStrafe)
                    .withRotationalRate(0.0);
                s_swerve.setControl(finalRequest);

                if (Math.abs(xError) < 0.05 && Math.abs(zError) < 0.05) 
                {
                    finished = true;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() 
    {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        var stopRequest = new SwerveRequest.RobotCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0);
        s_swerve.setControl(stopRequest);
    }
}
