// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0);
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(9);
  }

public void ShootFuel(double speed){
    shooterMotor.setControl(dutyCycle.withOutput(speed))
}

public void RunVelocity(double rps){
    voltageRequest.Velocity = rps;
    shooterMotor.setControl(voltageRequest); 
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