package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class SharedCommand extends SequentialCommandGroup {
    public SharedCommand(Robot robot, int turret, boolean extend, BooleanSupplier outtake) {
        super(
                new InstantCommand(() -> robot.bucket.close()),
                new WaitCommand(50),
                new InstantCommand(robot.intake::reverse),
                new InstantCommand(() -> robot.arm.armShared()),
                new InstantCommand(() -> robot.bucket.rest()),
                new WaitUntilCommand(() -> robot.arm.pos() > 50),
                new InstantCommand(() -> robot.turret.turn(turret)),
                new InstantCommand(robot.intake::stop),
                new WaitUntilCommand(() -> robot.arm.pos() > 250),
                new InstantCommand(() -> robot.arm.linkage(extend)),
                new WaitUntilCommand(() -> robot.arm.pos() > 650),
                new WaitUntilCommand(outtake),
                new InstantCommand(() -> robot.bucket.dump()),
                new InstantCommand(() -> robot.bucket.open()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.turret.middle()),
                new InstantCommand(() -> robot.arm.linkageIn()),
                new WaitCommand(100),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 350),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitUntilCommand(() -> robot.arm.pos() < 50),
                new InstantCommand(() -> robot.intake.start())
        );
    }

    public SharedCommand(Robot robot, int turret, boolean extend, BooleanSupplier outtake, Consumer<Boolean> done) {
        super(
                new InstantCommand(() -> robot.bucket.close()),
                new WaitCommand(50),
                new InstantCommand(robot.intake::reverse),
                new InstantCommand(() -> robot.arm.armShared()),
                new InstantCommand(() -> robot.bucket.rest()),
                new WaitUntilCommand(() -> robot.arm.pos() > 50),
                new InstantCommand(() -> robot.turret.turn(turret)),
                new InstantCommand(robot.intake::stop),
                new WaitUntilCommand(() -> robot.arm.pos() > 250),
                new InstantCommand(() -> robot.arm.linkage(extend)),
                new WaitUntilCommand(() -> robot.arm.pos() > 650),
                new WaitUntilCommand(outtake),
                new InstantCommand(() -> robot.bucket.dump()),
                new InstantCommand(() -> robot.bucket.open()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.turret.middle()),
                new InstantCommand(() -> robot.arm.linkageIn()),
                new WaitCommand(100),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 350),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitUntilCommand(() -> robot.arm.pos() < 50),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> done.accept(true))
        );
    }
}
