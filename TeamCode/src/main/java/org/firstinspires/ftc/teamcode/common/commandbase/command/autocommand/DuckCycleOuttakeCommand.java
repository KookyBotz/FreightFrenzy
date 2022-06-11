package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Arm;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class DuckCycleOuttakeCommand extends SequentialCommandGroup {
    public DuckCycleOuttakeCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.bucket.open()),
                new InstantCommand(() -> robot.bucket.dump_further()),
                new WaitCommand(700),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.turret.middle()),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new InstantCommand(() -> Arm.max_v = 8000),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 300),
                new InstantCommand(() -> robot.bucket.in())

        );
    }
}
