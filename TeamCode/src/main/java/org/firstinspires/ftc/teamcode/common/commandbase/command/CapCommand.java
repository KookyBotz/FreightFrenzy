package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadRetractCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class CapCommand extends SequentialCommandGroup {
    public CapCommand(Robot robot, BooleanSupplier next, Consumer<Boolean> done, Consumer<Boolean> done_capping) {
        super(
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.arm.setPos(700)),
                new WaitUntilCommand(() -> robot.arm.pos() > 350),
                new InstantCommand(() -> robot.bucket.dump_further()),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.arm.setPos(870)),
                new WaitCommand(500),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.bucket.close()),
                new WaitCommand(500),
                new InstantCommand(() -> robot.arm.setPos(435)),
                new InstantCommand(() -> robot.arm.linkage(() -> 1)),
                new WaitCommand(1000),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.arm.setPos(550)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.bucket.open()),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.arm.setPos(150)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new WaitUntilCommand(() -> robot.arm.pos() < 175),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitCommand(200),
                new InstantCommand(() -> robot.arm.armIn()),
                new InstantCommand(() -> done.accept(true)),
                new InstantCommand(() -> done_capping.accept(true))
        );
    }
}
