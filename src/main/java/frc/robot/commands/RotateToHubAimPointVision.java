// RotateToHubAimPointVision.java
package frc.robot.commands;


import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionAlignConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;


/**
 * Rotates robot to face the HUB center using Limelight MegaTag2 pose estimate.
 *
 * Key Fix vs your original:
 * - DO NOT "lock" to a single tag ID. Limelight will legitimately swap best tag IDs frame-to-frame,
 *   especially near edges of FOV / while rotating.
 * - Instead, accept aim if the MegaTag2 pose estimate includes ANY hub tag in rawFiducials.
 *
 * Translation (vx/vy) is passed through from the driver; this command only overrides omega.
 */
public class RotateToHubAimPointVision extends Command {


  private final CommandSwerveDrivetrain drivetrain;
  private final LimeLightSubsystem limelight;


  private final DoubleSupplier driverVx;
  private final DoubleSupplier driverVy;
  private final DoubleSupplier driverOmegaFallback;


  private final PIDController headingPID;


  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private double lastSeenTime = -1.0;
  private double atSetpointStart = -1.0;


  // Persist the last desired heading so we can coast through brief target dropouts.
  private Double desiredHeadingRad = null;


  public RotateToHubAimPointVision(
      CommandSwerveDrivetrain drivetrain,
      LimeLightSubsystem limelight,
      DoubleSupplier vx,
      DoubleSupplier vy,
      DoubleSupplier omegaFallback) {


    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.driverVx = vx;
    this.driverVy = vy;
    this.driverOmegaFallback = omegaFallback;


    headingPID = new PIDController(
        VisionAlignConstants.HEADING_kP,
        VisionAlignConstants.HEADING_kI,
        VisionAlignConstants.HEADING_kD);


    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(VisionAlignConstants.HEADING_TOLERANCE_DEG));
    headingPID.setIntegratorRange(VisionAlignConstants.HEADING_I_MIN, VisionAlignConstants.HEADING_I_MAX);


    addRequirements(drivetrain, limelight);
  }


  @Override
  public void initialize() {
    lastSeenTime = -1.0;
    atSetpointStart = -1.0;
    desiredHeadingRad = null;
    headingPID.reset();


    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    LimelightHelpers.setPipelineIndex(
        VisionAlignConstants.LIMELIGHT_NAME,
        alliance == DriverStation.Alliance.Red ? 0 : 1
    );
  }


  @Override
  public void execute() {
    final double now = Timer.getFPGATimestamp();


    // Pass-through driver translation; we only override rotation.
    final double vx = driverVx.getAsDouble();
    final double vy = driverVy.getAsDouble();
    double omegaCmd = driverOmegaFallback.getAsDouble();


    // Pull MegaTag2 pose (wpiBlue).
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionAlignConstants.LIMELIGHT_NAME);
    boolean hasPose = LimelightHelpers.validPoseEstimate(est) && est.tagCount > 0;


    // Accept if the pose was built using ANY hub tag.
    boolean seesHubTag = false;
    if (hasPose && est.rawFiducials != null) {
      for (LimelightHelpers.RawFiducial fid : est.rawFiducials) {
        if (VisionAlignConstants.HUB_TAG_IDS.contains(fid.id)) {
          seesHubTag = true;
          break;
        }
      }
    }


    // For debugging only (best tag id). Do NOT gate logic on this.
    int bestTid = (int) LimelightHelpers.getFiducialID(VisionAlignConstants.LIMELIGHT_NAME);
    boolean tv = LimelightHelpers.getTV(VisionAlignConstants.LIMELIGHT_NAME);


    // ============================
    // AUTO ALIGN (ROTATION ONLY)
    // ============================
    if (hasPose && seesHubTag) {
      lastSeenTime = now;


      Pose2d robotPose = est.pose;


      double dx = VisionAlignConstants.HUB_CENTER_WPI_BLUE.getX() - robotPose.getX();
      double dy = VisionAlignConstants.HUB_CENTER_WPI_BLUE.getY() - robotPose.getY();


      desiredHeadingRad = Math.atan2(dy, dx);


      double currentHeading = robotPose.getRotation().getRadians();
      omegaCmd = headingPID.calculate(currentHeading, desiredHeadingRad);
      omegaCmd = MathUtil.clamp(
          omegaCmd,
          -VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC,
           VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC);


      if (headingPID.atSetpoint()) {
        if (atSetpointStart < 0) atSetpointStart = now;
      } else {
        atSetpointStart = -1.0;
      }


    } else {
      // Brief dropout handling: keep turning toward the last computed desired heading.
      boolean withinGrace =
          lastSeenTime > 0 && (now - lastSeenTime) <= VisionAlignConstants.LOST_TARGET_GRACE_SEC;


      if (withinGrace && desiredHeadingRad != null) {
        // Use the latest pose if it's valid; otherwise, fall back to subsystem getter.
        Pose2d robotPose = hasPose ? est.pose : limelight.getBotPoseRightWpiBlue();
        double currentHeading = robotPose.getRotation().getRadians();


        omegaCmd = headingPID.calculate(currentHeading, desiredHeadingRad);
        omegaCmd = MathUtil.clamp(
            omegaCmd,
            -VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC,
             VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC);


      } else {
        // Full loss: stop the hold timer; omega remains the driver fallback.
        atSetpointStart = -1.0;
      }
    }


    // ============================
    // DRIVE
    // ============================
    drivetrain.setControl(
        driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omegaCmd)
    );


    // ============================
    // DEBUG
    // ============================
    SmartDashboard.putBoolean("Align/TV", tv);
    SmartDashboard.putNumber("Align/BestTID", bestTid);
    SmartDashboard.putBoolean("Align/HasPose", hasPose);
    SmartDashboard.putNumber("Align/TagCount", hasPose ? est.tagCount : 0);
    SmartDashboard.putBoolean("Align/SeesHubTag", seesHubTag);


    SmartDashboard.putBoolean("Align/AtSetpoint", headingPID.atSetpoint());
    SmartDashboard.putNumber("Align/OmegaCmd", omegaCmd);
    SmartDashboard.putNumber("Align/HeadingErrDeg", Math.toDegrees(headingPID.getPositionError()));


    if (desiredHeadingRad != null) {
      SmartDashboard.putNumber("Align/DesiredDeg", Math.toDegrees(desiredHeadingRad));
    }
  }


  @Override
  public void end(boolean interrupted) {
    // No special end behavior.
  }


  @Override
  public boolean isFinished() {
    if (atSetpointStart < 0) return false;


    double now = Timer.getFPGATimestamp();
    boolean recentlySeen =
        lastSeenTime > 0 && (now - lastSeenTime) <= VisionAlignConstants.LOST_TARGET_GRACE_SEC;


    if (!recentlySeen) return false;


    return (now - atSetpointStart) >= VisionAlignConstants.HOLD_TIME_SEC;
  }
}

