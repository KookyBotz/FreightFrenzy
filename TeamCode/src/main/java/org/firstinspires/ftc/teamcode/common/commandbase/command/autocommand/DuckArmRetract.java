package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class DuckArmRetract extends SequentialCommandGroup {
    public DuckArmRetract(Robot robot) {
        super(
                new InstantCommand(() -> robot.arm.setPos(350)),
                new WaitUntilCommand(() -> robot.arm.pos() > 300),
                new InstantCommand(() -> robot.turret.middle()),
                new WaitCommand(200),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new InstantCommand(() -> robot.arm.armIn())
        );
    }
}
