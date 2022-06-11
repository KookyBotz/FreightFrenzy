package org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Arm;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class DuckCycleExtendCommand extends SequentialCommandGroup {
    public DuckCycleExtendCommand(Robot robot, Alliance alliance){
        if(alliance == Alliance.BLUE){
            addCommands(
                    new WaitCommand(1000),
                    new InstantCommand(() -> robot.intake.stop()),
                    new InstantCommand(()->robot.bucket.close()),
                    new InstantCommand(()-> Arm.max_v = 3000),
                    new InstantCommand(()->robot.arm.setPos(470)),
                    new WaitUntilCommand(()->robot.arm.pos() > 350),
                    new InstantCommand(()->robot.arm.linkage(()->0.5)),
                    new InstantCommand(()->robot.turret.right())
            );
        }else{
            addCommands(
                    new WaitCommand(1000),
                    new InstantCommand(() -> robot.intake.stop()),
                    new InstantCommand(()->robot.bucket.close()),
                    new InstantCommand(()-> Arm.max_v = 3000),
                    new InstantCommand(()->robot.arm.setPos(470)),
                    new WaitUntilCommand(()->robot.arm.pos() > 350),
                    new InstantCommand(()->robot.arm.linkage(()->0.5)),
                    new InstantCommand(()->robot.turret.left())
            );
        }
    }
}
