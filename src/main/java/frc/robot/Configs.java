package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.shooter.ShooterConstants;

public final class Configs {
    private Configs() {}

    public static final class ShooterConfigs {


        public static TalonFXConfiguration shooterMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();


            config.Slot0.kP = ShooterConstants.kP;
            config.Slot0.kI = ShooterConstants.kI;
            config.Slot0.kD = ShooterConstants.kD;


            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;

           
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            config.CurrentLimits.StatorCurrentLimit = 40;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            return config;
        }
    }
}