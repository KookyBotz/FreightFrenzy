package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake extends SubsystemBase {
    public final MotorEx intake;

    public Intake(MotorEx i) {
        intake = i;
    }

    public void start() {
        intake.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.set(1);
    }

    public void reverse() {
        intake.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.set(-1);
    }

    public void stop() {
        intake.set(0);
        double pos = intake.motorEx.getCurrentPosition();
        int targetPos = (int) (pos + ((145.1 / 2.0 * 12.0 / 16.0) - (pos % (145.1 / 2.0 * 12.0 / 16.0))));
        intake.motorEx.setTargetPosition(targetPos);
        intake.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.motorEx.setPower(1);
    }

    public void toggle() {
        if (intake.get() == 0) {
            start();
        } else if (intake.get() == 1) {
            reverse();
        } else {
            stop();
        }
    }
}
