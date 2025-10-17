package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Adjust this import to match your actual subsystem package/class name.
import frc.robot.subsystems.Drivebase;

public class DriveDistance extends CommandBase {
  private final Drivebase drive;
  private final double targetMeters;
  private Pose2d startPose;

  // Tune these as needed
  private static final double kP = 1.5;        // proportional gain for distance
  private static final double kMaxSpeed = 1.0; // m/s clamp

  public DriveDistance(Drivebase drive, double meters) {
    this.drive = drive;
    this.targetMeters = meters;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startPose = drive.getPose();
  }

  @Override
  public void execute() {
    double traveled = startPose.getTranslation().getDistance(drive.getPose().getTranslation());
    double error = targetMeters - traveled;
    double speed = Math.copySign(Math.min(Math.abs(kP * error), kMaxSpeed), error);

    // Drive forward in robot-relative coordinates, no rotation.
    drive.drive(new Translation2d(speed, 0.0), 0.0, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drive.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
  }

  @Override
  public boolean isFinished() {
    double traveled = startPose.getTranslation().getDistance(drive.getPose().getTranslation());
    return traveled >= targetMeters;
  }
}
