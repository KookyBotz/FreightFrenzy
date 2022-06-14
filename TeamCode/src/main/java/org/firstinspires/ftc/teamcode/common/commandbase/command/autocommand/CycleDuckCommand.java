package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
public class CycleDuckCommand extends SequentialCommandGroup {
    public static double forward_power = 0.1;
    public static double duck_power = -0.32;
    public static long ms = 3000;

    public CycleDuckCommand(Robot robot) {
        double multiplier = 12.0 / robot.batteryVoltageSensor.getVoltage();
        forward_power *= multiplier;

        addCommands(
                new InstantCommand(() -> robot.drive.tankDrive(forward_power, forward_power)),
                new InstantCommand(() -> robot.intake.intake.set(duck_power)),
                new WaitCommand(ms),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.drive.tankDrive(0, 0))
        );
    }
}
