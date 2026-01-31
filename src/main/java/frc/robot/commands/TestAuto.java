package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.AlignToHubOdometry;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(CommandSwerveDrivetrain drivetrain) {
        
        Pose2d startingPose = new Pose2d(8.0, 1.0, new Rotation2d(0));

        Pose2d targetPose = new Pose2d(
            startingPose.getX() + 1.0, 
            startingPose.getY(),
            startingPose.getRotation()
        );

        addCommands(
            drivetrain.resetPoseCommand(startingPose),

            new AlignToHubOdometry(drivetrain, targetPose)
        );
    }
}