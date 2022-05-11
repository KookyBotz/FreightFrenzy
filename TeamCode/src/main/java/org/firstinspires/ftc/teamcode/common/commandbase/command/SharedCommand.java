package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;

public class SharedCommand extends SequentialCommandGroup {
    public SharedCommand(Robot robot, int turret, boolean extend, BooleanSupplier outtake) {
        super(
                new InstantCommand(() -> robot.intake.setPower(-1)),
                new InstantCommand(() -> robot.arm.armShared()),
                new InstantCommand(() -> robot.bucket.rest()),
                new WaitUntilCommand(() -> robot.arm.pos() > 400),
                new InstantCommand(() -> robot.turret.turn(turret)),
                new InstantCommand(()->robot.intake.setPower(0)),
                new WaitUntilCommand(() -> robot.arm.pos() > 600),
                new InstantCommand(() -> robot.arm.linkage(extend)),
                new WaitUntilCommand(() -> robot.arm.pos() > 1800),
                new WaitUntilCommand(outtake),
                new InstantCommand(() -> robot.bucket.dump()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.turret.middle()),
                new InstantCommand(() -> robot.bucket.in()),
                new InstantCommand(() -> robot.arm.linkageIn()),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 100),
                new InstantCommand(() -> robot.intake.setPower(1))

        );
    }
}
