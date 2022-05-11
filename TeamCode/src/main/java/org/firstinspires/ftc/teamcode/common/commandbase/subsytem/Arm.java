package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends SubsystemBase {
    private final DcMotorEx arm;
    private final Servo linkage;

    public static int in_position = 0;
    public static int shared_position = 1900;
    public static int high_position;
    public static int middle_position;
    public static int low_position;

    public static double linkage_in = 0.93;
    public static double linkage_out = 0.5;

    public Arm(DcMotorEx a, Servo l) {
        arm = a;
        linkage = l;
    }

    public void armIn() {
        arm.setTargetPosition(in_position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void armShared() {
        arm.setTargetPosition(shared_position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void linkageIn() {
        linkage.setPosition(linkage_in);
    }

    public void linkageOut() {
        linkage.setPosition(linkage_out);
    }

    public int pos() {
        return arm.getCurrentPosition();
    }

    public void linkage(boolean extend) {
        if (extend) {
            linkageOut();
            return;
        }
        linkageIn();
    }
}
