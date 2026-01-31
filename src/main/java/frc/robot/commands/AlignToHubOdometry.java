// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToHubOdometry extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain m_drivetrain;
    
    private PIDController xController, yController, rotController;
    private Pose2d m_targetPose;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToHubOdometry(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;
    xController = new PIDController(1.5, 0, 0);
    yController = new PIDController(1.5, 0, 0);
    rotController = new PIDController(1.5, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.

    rotController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetPose.getX());
    double ySpeed = yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetPose.getY());
    double rotation = rotController.calculate(m_drivetrain.getState().Pose.getRotation().getDegrees(), m_targetPose.getRotation().getDegrees());

    m_drivetrain.applyRequest(() ->
            drive.withVelocityX(xSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                .withVelocityY(ySpeed) // Drive left with negative X (left)
                .withRotationalRate(RotationsPerSecond.of(rotation).in(RadiansPerSecond)) // Drive counterclockwise with negative X (left)
        );
    
    SmartDashboard.putNumber("Velocity X: ", xSpeed);
    SmartDashboard.putNumber("Velocity Y: ", ySpeed);
    SmartDashboard.putNumber("Velocity rot: ", rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}