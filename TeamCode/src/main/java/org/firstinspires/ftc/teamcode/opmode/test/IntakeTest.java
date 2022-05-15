package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Intake;

@TeleOp
public class IntakeTest extends OpMode {
    private Intake intake;
    private boolean pA;
    @Override
    public void init() {
        MotorEx i = new MotorEx(hardwareMap, "intake");
        intake = new Intake(i);
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        if(a && !pA){
            intake.toggle();
        }
        pA = a;
    }
}
