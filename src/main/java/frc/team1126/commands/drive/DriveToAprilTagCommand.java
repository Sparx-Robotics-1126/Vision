package frc.team1126.commands.drive;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.Constants;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveToAprilTagCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonCamera camera;
    private final XboxController controller;
    private final Vision vision;
    private static final int TARGET_TAG_ID = 7;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private final double VISION_TURN_kP = 0.01;
    private final double VISION_DES_ANGLE_deg = 0.0;
    private final double VISION_STRAFE_kP = 0.5;
    private final double VISION_DES_RANGE_m = 0.25;


    public DriveToAprilTagCommand(SwerveSubsystem swerveSubsystem, PhotonCamera camera, XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.camera = camera;
        this.controller = controller;
        this.vision = this.swerveSubsystem.getVision();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        var maxSpeed = swerveSubsystem.getMaxSpeed();
        var maxAngularSpeed = swerveSubsystem.getMaxAngularVelocity();

        double forward = -controller.getLeftY() * maxSpeed;
        double strafe = -controller.getLeftX() * maxSpeed;
        double turn = -controller.getRightX() * maxAngularSpeed;


        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == TARGET_TAG_ID) {
                        targetVisible = true;
                    
                       targetRange= this.vision.getDistanceFromAprilTag(TARGET_TAG_ID);
                    
                        // Drive to the tag's position
//                        var pose = target.getBestCameraToTarget();
//                        var transform = target.getBestCameraToTarget();
//                        var pose = new Pose2d(transform.getTranslation().t

//                        swerveSubsystem.driveToPose(location.get().toPose2d());
                        break;
                    }
                }

            }
            if (targetVisible){
                // Driver wants auto-alignment to tag 7
                // And, tag 7 is in sight, so we can turn toward it.
                // Override the driver's turn and fwd/rev command with an automatic one
                // That turns toward the tag, and gets the range right.
                turn =
                        (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * maxAngularSpeed;
                forward =
                        (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * maxSpeed;
                        
            }
            // Command drivetrain motors based on target speeds
            SmartDashboard.putNumber("Forward", forward);
            SmartDashboard.putNumber("Strafe", strafe);
            SmartDashboard.putNumber("turn", turn);


            this.swerveSubsystem.drive(forward, strafe, turn);
        }
    }

    @Override
    public boolean isFinished() {
        // SmartDashboard.putNumber("Forward", 0);
        // SmartDashboard.putNumber("Strafe", 0);
        // SmartDashboard.putNumber("turn", 0);

        // SmartDashboard.putNumber("Range", 0);
        return false; // Run until interrupted
    }
}
