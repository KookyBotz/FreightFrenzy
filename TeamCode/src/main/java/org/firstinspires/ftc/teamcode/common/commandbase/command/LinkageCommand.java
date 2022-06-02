package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Arm;

import java.util.function.DoubleSupplier;

public class LinkageCommand extends CommandBase {
    private Arm arm;
    private DoubleSupplier supplier;

    public LinkageCommand(Arm arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
    }

    @Override
    public void execute() {
        arm.linkage(supplier);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
