// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToHubOdometry extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Pose2d m_targetPose;

  private final PIDController xController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController yController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController rotController = new PIDController(1.5, 0.0, 0.0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final double m_maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double m_maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  public AlignToHubOdometry(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;

    // Use radians end-to-end for rotation control
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Finish thresholds
    xController.setTolerance(0.05);                 // meters
    yController.setTolerance(0.05);                 // meters
    rotController.setTolerance(Math.toRadians(3.0)); // radians

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();

    SmartDashboard.putBoolean("AlignToHub/Running", true);
  }

  @Override
  public void execute() {
    Pose2d current = m_drivetrain.getState().Pose;

    // Treat PID outputs as velocities directly (m/s).
    // DO NOT multiply by kSpeedAt12Volts again (that was causing saturation/oscillation).
    double vx = MathUtil.clamp(
        xController.calculate(current.getX(), m_targetPose.getX()),
        -m_maxSpeedMps, m_maxSpeedMps
    );

    double vy = MathUtil.clamp(
        yController.calculate(current.getY(), m_targetPose.getY()),
        -m_maxSpeedMps, m_maxSpeedMps
    );

    // Rotation PID in radians; output is commanded omega (rad/s).
    double omega = MathUtil.clamp(
        rotController.calculate(current.getRotation().getRadians(), m_targetPose.getRotation().getRadians()),
        -m_maxAngularRateRadPerSec, m_maxAngularRateRadPerSec
    );

    m_drivetrain.setControl(
        drive.withVelocityX(vx)
             .withVelocityY(vy)
             .withRotationalRate(omega)
    );

    // Debug telemetry
    SmartDashboard.putNumber("AlignToHub/curX", current.getX());
    SmartDashboard.putNumber("AlignToHub/curY", current.getY());
    SmartDashboard.putNumber("AlignToHub/curDeg", current.getRotation().getDegrees());

    SmartDashboard.putNumber("AlignToHub/tgtX", m_targetPose.getX());
    SmartDashboard.putNumber("AlignToHub/tgtY", m_targetPose.getY());
    SmartDashboard.putNumber("AlignToHub/tgtDeg", m_targetPose.getRotation().getDegrees());

    SmartDashboard.putNumber("AlignToHub/vx_mps", vx);
    SmartDashboard.putNumber("AlignToHub/vy_mps", vy);
    SmartDashboard.putNumber("AlignToHub/omega_radps", omega);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when done/interrupted
    m_drivetrain.setControl(
        drive.withVelocityX(0.0)
             .withVelocityY(0.0)
             .withRotationalRate(0.0)
    );
    SmartDashboard.putBoolean("AlignToHub/Running", false);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
