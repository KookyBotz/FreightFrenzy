package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.command.SharedCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

@TeleOp
public class teleop extends CommandOpMode {
    private Robot robot;
    private BooleanSupplier outtake;
    private Consumer<Boolean> done;
    private DoubleSupplier linkage, arm;
    private boolean intake = true;
    private boolean extend;
    private double loop = 0;
    public Alliance alliance = Alliance.BLUE;

    private boolean intakeToggle = false;
    private boolean flag = true;

    private ElapsedTime timer;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        outtake = () -> gamepad1.right_bumper;
        done = (a) -> intake = a;
        linkage = () -> gamepad1.right_trigger;
        arm = () -> gamepad1.left_trigger;

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
        robot.intake.start();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        if(timer.seconds() > 30 && timer.seconds() < 60 && flag){
            gamepad1.rumble(500);
            flag = false;
        }

        if (timer.seconds() > 60 && timer.seconds() < 90 && !flag) {
            gamepad1.rumble(500);
            flag = true;
        }

        if (timer.seconds() > 90 && flag) {
            gamepad1.rumble(1000);
            flag = false;
        }

        super.run();
        robot.arm.loop();

        robot.drive.arcadeDrive(
                ether(-gamepad1.left_stick_y, 0.685, 0.06, 1),
                ether(gamepad1.right_stick_x, 0.685, 0.06, 1)
        );

        boolean a = gamepad1.a;
        if (a && !extend) {
            schedule(new SharedCommand(robot, alliance, outtake, linkage, arm, done));
        }
        extend = a;


        if (intake && robot.bucket.hasFreight()) {
            intake = false;
            schedule(new SharedCommand(robot, alliance, outtake, linkage, arm, done));
        }

        boolean x = gamepad1.x;
        if (x && !intakeToggle) {
            robot.intake.toggle();
        }
        intakeToggle = x;

        double time = System.currentTimeMillis();
        telemetry.addData("total loop time", time - loop);


        telemetry.addLine(String.format(Locale.ENGLISH, "left: %.2f, %.2f, %.2f right: %.2f, %.2f, %.2f intake: %.2f arm: %.2f",
                robot.left_back.motorEx.getCurrent(CurrentUnit.AMPS),
                robot.left_middle.motorEx.getCurrent(CurrentUnit.AMPS),
                robot.left_front.motorEx.getCurrent(CurrentUnit.AMPS),

                robot.right_back.motorEx.getCurrent(CurrentUnit.AMPS),
                robot.right_middle.motorEx.getCurrent(CurrentUnit.AMPS),
                robot.right_front.motorEx.getCurrent(CurrentUnit.AMPS),

                robot.i.motorEx.getCurrent(CurrentUnit.AMPS),
                robot.a.getCurrent(CurrentUnit.AMPS)
        ));

        telemetry.addData("intake ", robot.intake.intake.motorEx.getCurrentPosition());

        telemetry.addData("linkage ", gamepad1.right_trigger);
        telemetry.update();

        loop = System.currentTimeMillis();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double ether(double x, double p, double a_min, double a_max) {
        double v = p * Math.pow(x, 3) + (1 - p) * x;
        if (x > 0.001) {
            return a_max * (a_min + (1 - a_min)) * v;
        } else if (x < -0.001) {
            return a_max * (-a_min + (1 - a_min)) * v;
        } else {
            return 0;
        }
    }
}
