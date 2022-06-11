package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class DuckArmExtend extends SequentialCommandGroup {
    public DuckArmExtend(Robot robot, Alliance alliance) {
        super(
                new InstantCommand(() -> robot.arm.setPos(350)),
                new WaitUntilCommand(() -> robot.arm.pos() > 300),
                new InstantCommand(alliance == Alliance.BLUE ? () -> robot.turret.left() : () -> robot.turret.right()),
                new WaitCommand(200),
                new InstantCommand(() -> robot.arm.linkage(() -> 0.7)),
                new InstantCommand(() -> robot.arm.armIn())
        );
    }
}
