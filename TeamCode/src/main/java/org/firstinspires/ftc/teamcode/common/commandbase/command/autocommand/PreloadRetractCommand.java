package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class PreloadRetractCommand extends SequentialCommandGroup {
    public PreloadRetractCommand(Robot robot) {
        addCommands(
                new InstantCommand(() -> robot.bucket.open()),
                new InstantCommand(() -> robot.bucket.dump()),
                new WaitCommand(350),
                new InstantCommand(() -> robot.bucket.in()),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new InstantCommand(() -> robot.arm.armIn())
        );
    }
}
