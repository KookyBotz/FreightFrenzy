package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class PreloadRetractCommand extends SequentialCommandGroup {
    public PreloadRetractCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 300),
                new InstantCommand(() -> robot.bucket.in())

        );
    }
}
