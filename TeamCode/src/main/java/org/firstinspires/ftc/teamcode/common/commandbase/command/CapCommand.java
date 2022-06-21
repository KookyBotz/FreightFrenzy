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
    public CapCommand(Robot robot, BooleanSupplier next, Consumer<Boolean> done) {
        super(
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.arm.setPos(700)),
                new WaitUntilCommand(() -> robot.arm.pos() > 350),
                new InstantCommand(() -> robot.bucket.dump_further()),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.arm.setPos(850)),
                new WaitCommand(500),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.bucket.close()),
                new WaitCommand(500),
                new InstantCommand(() -> robot.arm.setPos(450)),
                new InstantCommand(() -> robot.arm.linkage(() -> 1)),
                new WaitCommand(1000),
                new WaitUntilCommand(next),
                new InstantCommand(() -> robot.arm.setPos(500)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.bucket.open()),
                new WaitUntilCommand(next),
                new PreloadRetractCommand(robot),
                new InstantCommand(()->done.accept(true))
        );
    }
}
