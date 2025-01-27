package frc.team1126.commands.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveToAprilTagCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonCamera camera;
    private static final int TARGET_TAG_ID = 7;

    public DriveToAprilTagCommand(SwerveSubsystem swerveSubsystem, PhotonCamera camera) {
        this.swerveSubsystem = swerveSubsystem;
        this.camera = camera;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == TARGET_TAG_ID) {
                        // Drive to the tag's position
//                        var pose = target.getBestCameraToTarget();
                        var transform = target.getBestCameraToTarget();
                        var pose = new Pose2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
                        swerveSubsystem.driveToPose(pose);
                        break;
                    }
                }

            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
