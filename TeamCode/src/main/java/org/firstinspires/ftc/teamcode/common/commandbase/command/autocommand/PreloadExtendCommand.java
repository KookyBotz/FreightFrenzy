package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.ff.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class PreloadExtendCommand extends SequentialCommandGroup {
    public PreloadExtendCommand(Robot robot, BarcodePipeline.BarcodePosition position) {
        if (position == BarcodePipeline.BarcodePosition.LEFT) { //bottom
            addCommands(
                    new InstantCommand(() -> robot.bucket.close()),
                    new InstantCommand(() -> robot.arm.setPos(770)),
                    new WaitUntilCommand(() -> robot.arm.pos() > 350),
                    new InstantCommand(() -> robot.arm.linkage(() -> 0.2))
            );
        } else if (position == BarcodePipeline.BarcodePosition.CENTER) {
            addCommands(
                    new InstantCommand(() -> robot.bucket.close()),
                    new InstantCommand(() -> robot.arm.setPos(700)),
                    new WaitUntilCommand(() -> robot.arm.pos() > 350),
                    new InstantCommand(() -> robot.arm.linkage(() -> 0.6))
            );
        } else {
            addCommands(
                    new InstantCommand(() -> robot.bucket.close()),
                    new InstantCommand(() -> robot.arm.setPos(500)),
                    new WaitUntilCommand(() -> robot.arm.pos() > 350),
                    new InstantCommand(() -> robot.arm.linkage(() -> 1))
            );
        }
    }
}
