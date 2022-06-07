package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Arm;

import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
    private Arm arm;
    private DoubleSupplier supplier;

    public ArmCommand(Arm arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
    }

    @Override
    public void execute() {
        arm.adjustArm(supplier);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
