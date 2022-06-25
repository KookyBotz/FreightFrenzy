package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AllianceHubPreCommand extends SequentialCommandGroup {
    public AllianceHubPreCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.bucket.close()),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitCommand(50),
                new InstantCommand(() -> robot.intake.reverse()),
                new WaitCommand(500),
                new InstantCommand(()->robot.intake.stop())
        );
    }
}
