package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
@Config
public class Drivetrain_PIDF extends OpMode {
    private Robot robot;


    public static double target = 0.0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.left.setRunMode(Motor.RunMode.VelocityControl);
        robot.right.setRunMode(Motor.RunMode.VelocityControl);
        robot.left.setVeloCoefficients(2, 0, 0);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robot.left.set(target);
        robot.right.set(target);

        telemetry.addData("left velo ", robot.left_encoder.getRawVelocity() / 2787.625);
        telemetry.addData("right velo ", robot.right_encoder.getRawVelocity() / 2787.625);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
