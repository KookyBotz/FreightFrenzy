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
                new WaitCommand(500),
                new InstantCommand(() -> robot.arm.setPos(150)),
                new WaitUntilCommand(()->robot.arm.pos()<175),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitCommand(200),
                new InstantCommand(()->robot.arm.armIn())
        );
    }
}
