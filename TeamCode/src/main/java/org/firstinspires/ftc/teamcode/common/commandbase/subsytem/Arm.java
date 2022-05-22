package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Arm extends SubsystemBase {
    public final DcMotorEx arm;
    private final Servo linkage;
    private final VoltageSensor batteryVoltageSensor;

    public static double linkage_in = 0.93;
    public static double linkage_out = 0.5;

    private final double p = 0.01;
    private final double d = 0.0005;
    private final double kcos = 0.2;
    private final double ticks_to_degrees = 700 / 180.0;

    private final PIDController controller;

    private double target = 0;

    public Arm(DcMotorEx a, Servo l, VoltageSensor b) {
        arm = a;
        linkage = l;

        controller = new PIDController(p, 0, d);
        controller.setPID(p, 0, d);
        this.batteryVoltageSensor = b;
    }

    public void loop() {
        int pos = arm.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_to_degrees)) * kcos;

        double power = (pid + ff) * batteryVoltageSensor.getVoltage() / 12.0;

        arm.setPower(power);
    }

    public void armIn() {
        target = 0;
    }

    public void armShared() {
        target = 700;
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
