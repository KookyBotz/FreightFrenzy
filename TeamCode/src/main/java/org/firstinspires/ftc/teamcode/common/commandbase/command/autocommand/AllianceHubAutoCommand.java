package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AllianceHubAutoCommand extends SequentialCommandGroup {
    public AllianceHubAutoCommand(Robot robot) {
        super(
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.bucket.close()),
                new InstantCommand(() -> robot.arm.setPos(550)),
                new WaitUntilCommand(() -> robot.arm.pos() > 350),
                new InstantCommand(() -> robot.arm.linkage(() -> 0.9))
        );
    }
}
