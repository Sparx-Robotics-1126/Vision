package frc.team1126.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotState;
// import frc.robot.constants.Constants;
// import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
// import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
// import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
// import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
// import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.team1126.subsystems.SwerveSubsystem;
import swervelib.parser.SwerveDriveConfiguration;

public class LinearDriveToPose extends Command {
    private final SwerveSubsystem swerveDrive;// = SwerveDrive.getInstance();

    private final TrapezoidProfile xTranslationalMotionProfile;
    private final PIDController xTranslationalFeedbackController;

    private State xTranslationalGoal = new State();
    private State currentXTranslationalSetpoint = new State();

    private final TrapezoidProfile yTranslationalMotionProfile;
    private final PIDController yTranslationalFeedbackController;

    private State yTranslationalGoal = new State();
    private State currentYTranslationalSetpoint = new State();

    private final TrapezoidProfile rotationalMotionProfile;
    private final PIDController rotationalFeedbackController;

    private State rotationalGoal = new State(0, 0);
    private State currentRotationalSetpoint = new State(0, 0);

    private double previousTimestamp = Timer.getTimestamp();

    protected Supplier<Pose2d> targetPose;
    protected Supplier<ChassisSpeeds> endVelo;

    private final SwerveDriveConfiguration drivetrainConfig;
        private final double maxTranslationalVelocity = 4.0;
    private final double maxTranslationalAcceleration = 3.5;
    private final double maxAngularVelocity = 3.7;
    private final double maxAngularAcceleration = 12.0;
    private final double maxAutoModuleVelocity = 5.0;

    // This is the blue alliance pose!
    // field relative velo and pose and velocity
    public LinearDriveToPose(SwerveSubsystem swerveDrive, Supplier<Pose2d> targetPose, Supplier<ChassisSpeeds> endVelo) {
        this.swerveDrive = swerveDrive;
        drivetrainConfig = swerveDrive.getSwerveDriveConfiguration();


        this.targetPose = targetPose;
        this.endVelo = endVelo;

        this.xTranslationalMotionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        maxTranslationalVelocity,
                        maxTranslationalAcceleration
                )
        );

        this.yTranslationalMotionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        maxTranslationalVelocity,
                        maxTranslationalAcceleration
                )
        );

        this.xTranslationalFeedbackController = getAutoAlignProfiledTranslationController();
        this.yTranslationalFeedbackController = getAutoAlignProfiledTranslationController();

        this.rotationalMotionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        maxAngularVelocity,
                        maxAngularAcceleration
                )
        );

        this.rotationalFeedbackController = getAutoAlignProfiledRotationController();

        System.out.println("NEW COMMAND");
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        System.out.println("init");

        this.xTranslationalGoal = new State(
                targetPose.get().getX(),
                endVelo.get().vxMetersPerSecond
        );

        this.yTranslationalGoal = new State(
                targetPose.get().getY(),
                endVelo.get().vyMetersPerSecond
        );

        // rotation wrapping
        double desiredRotationRad = MathUtil.angleModulus(targetPose.get().getRotation().getRadians());
        double rotationError = MathUtil.inputModulus(
                desiredRotationRad - swerveDrive.getPose().getRotation().getRadians(),
                -Math.PI, Math.PI
        );

        this.rotationalGoal = new State(
                swerveDrive.getPose().getRotation().getRadians() + rotationError,
                endVelo.get().omegaRadiansPerSecond
        );


        currentXTranslationalSetpoint = new State(
                swerveDrive.getPose().getX(),
                swerveDrive.getFieldVelocity().vxMetersPerSecond
        );

        currentYTranslationalSetpoint = new State(
                swerveDrive.getPose().getY(),
                swerveDrive.getFieldVelocity().vyMetersPerSecond
        );

        currentRotationalSetpoint = new State(
                swerveDrive.getPose().getRotation().getRadians(),
                swerveDrive.getFieldVelocity().omegaRadiansPerSecond
        );

        previousTimestamp = Timer.getTimestamp();

        Logger.recordOutput("LinearDriveToPose/targetPose.get()", targetPose.get());

        xTranslationalFeedbackController.reset();
        yTranslationalFeedbackController.reset();
        rotationalFeedbackController.reset();
    }

    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - previousTimestamp;

        currentXTranslationalSetpoint = xTranslationalMotionProfile.calculate(
                dt,
                currentXTranslationalSetpoint,
                xTranslationalGoal
        );
        Logger.recordOutput("LinearDriveToPose/currentXTranslationalSetpointVelo", currentXTranslationalSetpoint.velocity);

        currentYTranslationalSetpoint = yTranslationalMotionProfile.calculate(
                dt,
                currentYTranslationalSetpoint,
                yTranslationalGoal
        );
        Logger.recordOutput("LinearDriveToPose/currentYTranslationalSetpointVelo", currentYTranslationalSetpoint.velocity);
        Logger.recordOutput("LinearDriveToPose/currentYTranslationalSetpointPose", currentYTranslationalSetpoint.position);

        currentRotationalSetpoint = rotationalMotionProfile.calculate(
                dt,
                currentRotationalSetpoint,
                rotationalGoal
        );

        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(0, 0, 0);

        calculatedSpeeds.vxMetersPerSecond =
                xTranslationalFeedbackController.calculate(
                        swerveDrive.getPose().getX(),
                        currentXTranslationalSetpoint.position
                ) +
                        Math.max(
                                endVelo.get().vxMetersPerSecond,
                                currentXTranslationalSetpoint.velocity
                        );

        calculatedSpeeds.vyMetersPerSecond =
                xTranslationalFeedbackController.calculate(
                        swerveDrive.getPose().getY(),
                        currentYTranslationalSetpoint.position
                ) +
                        Math.max(
                                endVelo.get().vyMetersPerSecond,
                                currentYTranslationalSetpoint.velocity
                        );

        calculatedSpeeds.omegaRadiansPerSecond =
                rotationalFeedbackController.calculate(
                        swerveDrive.getPose().getRotation().getRadians(),
                        currentRotationalSetpoint.position
                ) +
                        Math.max(
                                endVelo.get().omegaRadiansPerSecond,
                                currentRotationalSetpoint.velocity
                        );

        swerveDrive.drive(calculatedSpeeds);

        previousTimestamp = Timer.getTimestamp();
    }

    @Override
    public boolean isFinished() {
        boolean poseAligned =
                swerveDrive.getPose().getTranslation().getDistance(targetPose.get().getTranslation()) <=
                        getAutoAlignTranslationTolerance() &&
                        Math.abs(swerveDrive.getPose().getRotation().getRadians() - targetPose.get().getRotation().getRadians()) <=
                                getAutoAlignRotationTolerance();

        boolean speedsAligned =
                endVelo.get().vxMetersPerSecond == 0 && endVelo.get().vyMetersPerSecond == 0 && endVelo.get().omegaRadiansPerSecond == 0 ?
                        Math.abs(Math.hypot(
                                swerveDrive.getFieldVelocity().vxMetersPerSecond,
                                swerveDrive.getFieldVelocity().vyMetersPerSecond
                        )) <= getAutoAlignTranslationVeloTolerance() &&
                                Math.abs(swerveDrive.getFieldVelocity().omegaRadiansPerSecond) <= getAutoAlignRotationVeloTolerance() :
                        true;

        return poseAligned && speedsAligned;
    }

        public PIDController getAutoAlignProfiledTranslationController() {
        PIDController p = new PIDController(2, 0, 0);
        p.setTolerance(Math.sqrt(0.05));

        return p;
    }

    public PIDController getAutoAlignProfiledRotationController() {
        PIDController p = new PIDController(1, 0, 0);
        p.setTolerance(Math.toRadians(5));
        p.enableContinuousInput(-Math.PI, Math.PI);

        return p;
    }
    public double getAutoAlignTranslationTolerance() {
        return 0.05;
    }
    public double getAutoAlignRotationTolerance() {
        return Math.toRadians(5);
    }

        public double getAutoAlignTranslationVeloTolerance() {
        return 0.03;
    }

        public double getAutoAlignRotationVeloTolerance() {
        return Math.toRadians(3);
    }
//    public ChassisSpeeds getFieldRelativeSpeeds() {
//        return ChassisSpeeds.fromRobotRelativeSpeeds(
//            swerveDrive.getKinematics().toChassisSpeeds(null)
//            robotRelativeVelocity, lastGyroOrientation.toRotation2d());
//      }
}