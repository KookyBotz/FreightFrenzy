package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.ff.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public class PreloadExtendCommand extends ParallelCommandGroup {
    public PreloadExtendCommand(Robot robot, BarcodePipeline.BarcodePosition analysis, Alliance alliance, DifferentialDriveOdometry odometry, Telemetry telemetry) {
        double multiplier = alliance == Alliance.BLUE ? 1 : -1;
        if (analysis == BarcodePipeline.BarcodePosition.LEFT) { //bottom
            addCommands(
                    new DrivetrainCommand(new Pose(-20, 5 * multiplier, -45 * multiplier), robot, odometry, telemetry),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.bucket.close()),
                            new InstantCommand(() -> robot.arm.setPos(770)),
                            new WaitUntilCommand(() -> robot.arm.pos() > 350),
                            new InstantCommand(() -> robot.arm.linkage(() -> 0.15))
                    )
            );
        } else if (analysis == BarcodePipeline.BarcodePosition.CENTER) {
            addCommands(
                    new DrivetrainCommand(new Pose(-20, 5 * multiplier, -45 * multiplier), robot, odometry, telemetry),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.bucket.close()),
                            new InstantCommand(() -> robot.arm.setPos(675)),
                            new WaitUntilCommand(() -> robot.arm.pos() > 350),
                            new InstantCommand(() -> robot.arm.linkage(() -> 0.6))
                    )
            );
        } else {
            addCommands(
                    new DrivetrainCommand(new Pose(-20, 5 * multiplier, -45 * multiplier), robot, odometry, telemetry),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.bucket.close()),
                            new InstantCommand(() -> robot.arm.setPos(550)),
                            new WaitUntilCommand(() -> robot.arm.pos() > 350),
                            new InstantCommand(() -> robot.arm.linkage(() -> 0.835))
                    )
            );
        }
    }
}
