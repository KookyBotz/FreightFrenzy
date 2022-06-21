package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class AllianceHubCommand extends SequentialCommandGroup {
    public AllianceHubCommand(Robot robot, BooleanSupplier outtake, Consumer<Boolean> done) {
        super(
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.arm.setPos(485)),
                new WaitUntilCommand(() -> robot.arm.pos() > 350),
                new InstantCommand(() -> robot.arm.linkage(() -> 0.7)),
                new WaitUntilCommand(outtake),
                new InstantCommand(() -> robot.bucket.open()),
                new InstantCommand(() -> robot.bucket.dump_further()),
                new WaitCommand(500),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.arm.linkage(() -> 0)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 350),
                new WaitCommand(150),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitUntilCommand(() -> robot.arm.pos() < 50),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> done.accept(true))
        );
    }
}
