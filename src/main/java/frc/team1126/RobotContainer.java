
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1126.commands.drive.LinearDriveToPose;
import frc.team1126.subsystems.LEDs;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.OperatorConstants;
//import frc.team1126.commands.LEDManager;
import frc.team1126.commands.drive.AbsoluteDriveAdv;
import frc.team1126.commands.drive.DriveToAprilTagCommand;
import frc.team1126.commands.drive.DriveToClosestLeftBranchPoseCommand;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.Vision;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Meter;

public class RobotContainer {

//    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    final static SendableChooser<Command> chooser = new SendableChooser<>();

    final static SendableChooser<Command> m_chooser = new SendableChooser<>();

    CommandXboxController driverController = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
    // CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

    public static SwerveSubsystem swerve = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve"));

 // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(swerve,
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                                               driverController.getHID()::getYButtonPressed,
                                                                                               driverController.getHID()::getAButtonPressed,
                                                                                               driverController.getHID()::getXButtonPressed,
                                                                                               driverController.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverController.getRightX() *-1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
  driverController::getRightY)
                                                           .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = swerve.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot


  Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                   () -> -driverController.getLeftY(),
                                                                   () -> -driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                        driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);
    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */


    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerve.getSwerveDrive(),
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX())
            .withControllerRotationAxis(() -> driverController.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() ->
                            Math.sin(
                                    driverController.getRawAxis(
                                            2) *
                                            Math.PI) *
                                    (Math.PI *
                                            2),
                    () ->
                            Math.cos(
                                    driverController.getRawAxis(
                                            2) *
                                            Math.PI) *
                                    (Math.PI *
                                            2))
            .headingWhile(true);
  Command driveFieldOrientedDirectAngleSim = swerve.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

//private IAddressableLEDHelper[] leds;
//  private MultiFunctionLED multifucntion;
//  private LEDManager ledManager;

    private final LEDs ledSubsystem = new LEDs(0, 300);

    PhotonCamera m_noteCamera;

    public RobotContainer() {
      
//        configureLEDs();
        m_noteCamera = new PhotonCamera("front");
        /* REGISTER PATHPLANNER COMMANDS HERE */
     
    // Command test = swerve.driveCommand(
    //             () -> MathUtil.clamp(MathUtil.applyDeadband(-driverController.getLeftY(), .1), -1,
    //                     1),
    //             () -> MathUtil.clamp(MathUtil.applyDeadband(-driverController.getLeftX(), .1), -1,
    //                     1),
    //             () -> driverController.getRightX());

        //OTHER COMMANDS
        //NamedCommands.registerCommand("limelightTarget", new LLRotationAlignCommand(m_swerve).withTimeout(1.5));
        // DriverStation.silenceJoystickConnectionWarning(true);

//        swerve.setDefaultCommand(test );
        // m_swerve = new SwerveSubsystem(
                // new File(Filesystem.getDeployDirectory(), "swerve"));

        // Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        //         () -> MathUtil.clamp(MathUtil.applyDeadband(m_driver.getLeftY()*.75, .1), -1,
        //                 1),
        //         () -> MathUtil.clamp(MathUtil.applyDeadband(m_driver.getLeftX()*.75, .1), -1,
        //                 1),
        //         () -> m_driver.getRightX());


       
        configureChooser();

        configureDriverBindings();

    }

    private void configureDriverBindings() {

        Command driveFieldOrientedDirectAngle      = swerve.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = swerve.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = swerve.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard      = swerve.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = swerve.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = swerve.driveWithSetpointGeneratorFieldRelative(                driveDirectAngleKeyboard);

        swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        if (RobotBase.isSimulation())
        {
            swerve.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else
        {
            swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (DriverStation.isTest())
        {
            swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driverController.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
            driverController.y().whileTrue(swerve.driveToDistanceCommand(1.0, 0.2));
            driverController.start().onTrue((Commands.runOnce(swerve::zeroGyro)));
            driverController.back().whileTrue(swerve.centerModulesCommand());
            driverController.leftBumper().onTrue(Commands.none());
            driverController.rightBumper().onTrue(Commands.none());
        } else {
            driverController.leftTrigger().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
            driverController.rightTrigger().onChange(new InstantCommand(() -> swerve.zeroGyroWithAlliance()));
            driverController.a().onTrue((Commands.runOnce(swerve::zeroGyro)));
//        m_driver.x().onTrue(Commands.runOnce(m_swerve::addFakeVisionReading));
            // m_driver.b().whileTrue(
            //     m_swerve.driveToPose(
            //       new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
            //                       );
            //                     //   m_driver.y().whileTrue(m_swerve.aimAtSpeaker(2));
            //                       m_driver.start().whileTrue(Commands.none());
            //                       m_driver.back().whileTrue(Commands.none());
            //                       m_driver.leftBumper().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());

            //                       m_driver.rightBumper().onTrue(Commands.none());
            // m_driver.y().onTrue(new ChaseLEDColorCommand(ledSubsystem, new Color8Bit(255, 0, 0), 10)); // Chasing red color
            // m_driver.x().onTrue(new GradientCommand(ledSubsystem, new Color8Bit(0,0,255), new Color8Bit(255,0,0)));
            // m_driver.a().onTrue(new RainbowCommand(ledSubsystem));
            // m_driver.b().onTrue(new PulseCommand(ledSubsystem, new Color8Bit(0, 255, 0), 7));
            //m_driver.leftBumper().onTrue(new SetSolidColorCommand(ledSubsystem, new Color8Bit(0,0,255)));
            driverController.leftBumper().whileTrue(new DriveToClosestLeftBranchPoseCommand(swerve));
            driverController.x().whileTrue(new LinearDriveToPose(swerve, () -> swerve.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
            //  driverController.x().whileTrue(new DriveToAprilTagCommand(swerve, m_noteCamera, driverController.getHID()));
            driverController.b().whileTrue(swerve.driveToPose(new Pose2d(new Translation2d(Meter.of(16.4), Meter.of(4.4)), Rotation2d.fromDegrees(180))));
            driverController.y().whileTrue(swerve.driveToPose(new Pose2d(new Translation2d(13, 4), Rotation2d.fromDegrees(180))));
            driverController.leftBumper().whileTrue(swerve.driveToPose(swerve.getClosestLeftBranchPose()));
            // driverController.leftBumper().whileTrue(new LinearDriveToPose(swerve, () -> swerve.getClosestLeftBranchPose(),() ->  new ChassisSpeeds()));
            // driverController.rightBumper().whileTrue(new LinearDriveToPose(swerve, () -> swerve.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
        }
    }
   
//public void configureLEDs() {
//    multifucntion = new MultiFunctionLED(
//      new BatteryLED(300));
//
//    leds = new IAddressableLEDHelper[]{multifucntion};
//
//    ledManager = new LEDManager(0, leds);
//    ledManager.schedule();
//  }

    double getXSpeed() {
        int pov = driverController.getHID().getPOV();
        double finalX;

        if (pov == 0)
            finalX = -0.05;
        else if (pov == 180)
            finalX = 0.05;
        else if (Math.abs(driverController.getLeftY()) <= 0.1)
            finalX = 0.0;
        else
            finalX = driverController.getLeftY() * 0.75 * (1.0 + driverController.getLeftTriggerAxis());

        return finalX;
    }

    public double getYSpeed() {
        int pov = driverController.getHID().getPOV();

        double finalY;
        if (pov == 270 || pov == 315 || pov == 225)
            finalY = -0.05;
        else if (pov == 90 || pov == 45 || pov == 135)
            finalY = 0.05;
        else if (Math.abs(driverController.getLeftX()) <= 0.1)
            finalY = 0.0;
        else
            finalY = driverController.getLeftX() * 0.75 * (1.0 + driverController.getLeftTriggerAxis());

        return finalY;
    }

    public void configureChooser() {

        m_chooser.setDefaultOption("straightAuto", new PathPlannerAuto("straightAuto"));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *\
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // swerve.zeroGyro();
        //
        // // m_driveSubsystem.setHeading(180);
        // Timer.delay(0.05);
        // // the command to be run in autonomous

        // return _chooser.getSelected();
        return m_chooser.getSelected();
        // return swerve.getAutonomousCommand(_chooser.().getName(), true);

    }

 

    public void EndGameRumble() {

        // if(DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 18) {
        //     m_candleSubsystem.setLEDState(LEDState.PURPLE);
        // }

        // if (DriverStation.getMatchTime() < SwerveConstants.ENDGAME_SECONDS 
        //         && DriverStation.getMatchTime() > SwerveConstants.STOP_RUMBLE_SECONDS) {
            
        //     m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);

        // } else {
        //     m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

        // }
    }

    //method for operator to know whether or not the shooter is up to speed for any given distance
    public void upToSpeedRumble() {
        // if(m_shooter.isMotorUpToSpeed()) {
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,1);
        // } 
        // m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
    }

        public static void getSmartDashboardTable() {
        ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
        GenericEntry test = tab.add("test variable", 1).getEntry();
    }

}
