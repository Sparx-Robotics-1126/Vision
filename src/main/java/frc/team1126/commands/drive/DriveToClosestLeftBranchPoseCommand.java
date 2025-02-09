package frc.team1126.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestLeftBranchPoseCommand extends Command {
    private final SwerveSubsystem swerve;
    Pose2d targetPose;
    public DriveToClosestLeftBranchPoseCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetPose = swerve.getClosestLeftBranchPose();
        swerve.driveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerve.getPose();
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleDifference = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());

        // Define a threshold for considering the robot to be at the target pose
        double positionThreshold = 0.1; // meters
        double angleThreshold = 5.0; // degrees

        return distance < positionThreshold && angleDifference < angleThreshold;
    }
}