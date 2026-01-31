// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Configs;
import frc.robot.Configs.ShooterConfigs;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.shooterMotorOneID);
    
    configureMotors();
  }

  public void configureMotors() {
    shooterMotor.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
  }

  public void shootFuel(double speed) {
    shooterMotor.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotor.setControl(voltageRequest); 
  }
  
  public void runVelocityTorqueFOC(double rps) {
    shooterMotor.setControl(
      velocityTorqueRequest.withVelocity(rps)
    );
  }
  

  public void printVoltageOutput() {
    double motorVoltage = shooterMotor.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotor.getSupplyCurrent().getValueAsDouble());
  }

  public void printRPM() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    double shooterRPM = (motorRPS / 6.0) * 60.0;
    SmartDashboard.putNumber("Shooter Motor RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}