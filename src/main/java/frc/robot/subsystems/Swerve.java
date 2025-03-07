package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem 
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    LimelightHelpers.PoseEstimate llMeasurement;

    private Pose2d onTheFlyDestination = new Pose2d();
    private final Field2d field = new Field2d();

    private double speedFactor;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine
    (
        new SysIdRoutine.Config
        (
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism
        (
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine
    (
        new SysIdRoutine.Config
        (
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism
        (
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine
    (
        new SysIdRoutine.Config
        (
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism
        (
            output -> 
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public boolean isWithinTolerance(double tolerance) 
    {
        // Get the current robot position
        Pose2d currentPose = getState().Pose;
    
        // Get the destination position
        Pose2d targetPose = onTheFlyDestination;
    
        // If no target is set, return false
        if (targetPose == null) 
        {
            return false;
        }
    
        // Calculate Euclidean distance between current position and target
        double distance = Math.sqrt
        (
            Math.pow(targetPose.getX() - currentPose.getX(), 2) +
            Math.pow(targetPose.getY() - currentPose.getY(), 2)
        );
    
        // Return true if within tolerance
        return distance <= tolerance;
    }

    public boolean isRotationComplete() 
    {
        // Get the current rotation
        double currentAngle = getState().Pose.getRotation().getDegrees();
    
        // Get the desired rotation from the target pose
        double targetAngle = onTheFlyDestination.getRotation().getDegrees();
    
        // Define the acceptable tolerance for rotation (adjust as needed)
        double rotationTolerance = 5.0; // degrees // TODO - Tune ?
    
        // Return true if within tolerance
        return Math.abs(targetAngle - currentAngle) <= rotationTolerance;
    }

    public void setDestination(Pose2d destination)
    {
        onTheFlyDestination = destination;
        System.out.println("Set Destination" + destination);
    }

    public Pose2d getDestination()
    {        
        //isAlgae = ((s_elevator.getHeightPreset() == Constants.Elevator.HeightPresets.A1) || (s_elevator.getHeightPreset() == Constants.Elevator.HeightPresets.A2));
        
        /*
        if(isAlgae) // Change requested Posed2d to the Algae Cleaning Pose2d if Height is set to Algae
        {
            if(onTheFlyDestination == Constants.Destinations.A || onTheFlyDestination == Constants.Destinations.B)
            {
                return Constants.Destinations.AB;
            }
            else if(onTheFlyDestination == Constants.Destinations.C || onTheFlyDestination == Constants.Destinations.D)
            {
                return Constants.Destinations.CD;
            }
            else if(onTheFlyDestination == Constants.Destinations.E || onTheFlyDestination == Constants.Destinations.F)
            {
                return Constants.Destinations.EF;
            }
            else if(onTheFlyDestination == Constants.Destinations.G || onTheFlyDestination == Constants.Destinations.H)
            {
                return Constants.Destinations.GH;
            }
            else if(onTheFlyDestination == Constants.Destinations.I || onTheFlyDestination == Constants.Destinations.J)
            {
                return Constants.Destinations.IJ;
            }
            else if(onTheFlyDestination == Constants.Destinations.K || onTheFlyDestination == Constants.Destinations.L)
            {
                return Constants.Destinations.KL;
            }
        }
    */
        System.out.println("GET Destination" + onTheFlyDestination);
        return onTheFlyDestination;
    }
    
    public double getSpeedFactor(double elevatorHeight, boolean climbEnabled) 
    {
        if (elevatorHeight > Constants.Elevator.HeightPresets.handoffHeight + .5) // 0.5 inch saftety   // If height is above handoff height, drive at 1/2 speed
        {
            speedFactor = .5;
        }
        else if(climbEnabled) // If climb is enabled, drive at 1/2 speed
        {
            speedFactor = .2;
        }
        else    // If climber is not enabled and height is at or below handoff height, drive at full speed
        {
            speedFactor = 1;
        }
        return speedFactor;
    }
    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Swerve
    (
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) 
    {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve
    (
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) 
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve
    (
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) 
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        configureAutoBuilder();
        
        // Publish the Field2d object to SmartDashboard for visualization
        SmartDashboard.putData("Field", field);
    }

        private void configureAutoBuilder() 
        {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(5, 0, 0),    // .64, 0, 0 -3/6/25  // CT: 5, 0, 0
                    // PID constants for rotation
                    new PIDConstants(5, 0, 0)      // 25, 0, 0 -3/6/25  // CT: 5, 0, 0
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) 
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() 
    {
        updateVisionOdometry();

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) 
        {
            DriverStation.getAlliance().ifPresent(allianceColor -> 
            {
                setOperatorPerspectiveForward
                (
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() 
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> 
        {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement
    (
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) 
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }


    /*
    public Command driveToSetPose(Supplier<Pose2d> suppliedPose) 
    {
        Pose2d pose = suppliedPose.get();
        System.out.println("Pathfinding to: " + pose);
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(2.2352, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(720)), 0.0);
    }
*/
    
    public Command driveToPose(Pose2d pose)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(2.2352, 4.0,Units.degreesToRadians(360) , Units.degreesToRadians(720));   // TODO - Tune
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(pose, constraints,0.0);
    }

    /*
    public Command driveToSetPose()
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(2.2352, 4.0,Units.degreesToRadians(360) , Units.degreesToRadians(720));   // TODO - Tune
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(onTheFlyDestination, constraints,0.0);
    }
        */


        /*
    public Command driveToSetPose(Supplier<Pose2d> suppliedPose)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(2.2352, 4.0,Units.degreesToRadians(360) , Units.degreesToRadians(720));   // TODO - Tune
        
        
        System.out.println("Supplied pose is: " + suppliedPose.get().getX() + suppliedPose.get().getY());
        

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(suppliedPose.get(), constraints,0.0);
    }
        */


    public void poseToLL() 
    { 
        if(llMeasurement != null && llMeasurement.tagCount > 0)
        {
            resetPose(llMeasurement.pose);
        }
    }

    private void updateVisionOdometry()
    {
        double omegaRPS = Units.radiansToRotations(getState().Speeds.omegaRadiansPerSecond);
        double headingDeg = getState().Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("HEADING", headingDeg);
        
        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRPS < 2.0) 
        { 
            System.out.println(llMeasurement.pose);
            
            addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        }
    }
}
