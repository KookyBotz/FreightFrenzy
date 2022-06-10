package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class PreloadDumpCommand extends SequentialCommandGroup {
    public PreloadDumpCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.bucket.open()),
                new InstantCommand(() -> robot.bucket.dump()),
                new WaitCommand(350)
        );
    }
}
