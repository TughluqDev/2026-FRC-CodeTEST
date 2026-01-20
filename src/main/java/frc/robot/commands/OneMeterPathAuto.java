package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OneMeterPathAuto extends SequentialCommandGroup {
    private static final PathConstraints kConstraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI);

    public OneMeterPathAuto(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            Commands.defer(
                () -> {
                    Pose2d currentPose = drivetrain.getPose();
                    Translation2d delta = new Translation2d(1.0, 0.0).rotateBy(currentPose.getRotation());
                    Pose2d targetPose = new Pose2d(
                        currentPose.getTranslation().plus(delta),
                        currentPose.getRotation()
                    );
                    return AutoBuilder.pathfindToPose(targetPose, kConstraints, 0.0);
                },
                Set.of(drivetrain)
            )
        );
    }
}
