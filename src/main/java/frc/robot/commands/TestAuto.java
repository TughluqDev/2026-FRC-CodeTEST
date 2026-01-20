package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(CommandSwerveDrivetrain drivetrain) {
        Pose2d targetPose = new Pose2d(
            drivetrain.getPose().getX() + 1.0,
            drivetrain.getPose().getY(),
            drivetrain.getPose().getRotation()
        );

        addCommands(drivetrain.findPathToPose(targetPose));
    }
}
