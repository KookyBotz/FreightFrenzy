package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.command.SharedCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

@TeleOp
public class teleop extends CommandOpMode {
    private Robot robot;
    private BooleanSupplier outtake;
    private Consumer<Boolean> done;
    private boolean intake = true;
    private boolean extend;
    private double loop = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        outtake = () -> gamepad1.b;
        done = (a) -> intake = a;

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
    }

    @Override
    public void run() {
        super.run();
        double update = System.currentTimeMillis();
        telemetry.addData("command scheduler update time ", update - loop);

        robot.drive.arcadeDrive(-gamepad1.left_stick_y, Math.pow(gamepad1.right_stick_x, 3));

        double drive = System.currentTimeMillis();
        telemetry.addData("drive update time ", drive - update);


        robot.arm.loop();

        double arm = System.currentTimeMillis();
        telemetry.addData("arm update time ", arm - drive);

//
//        Rotation2d imu = new Rotation2d(robot.imu.getAngularOrientation().firstAngle);
//        double right_position = robot.right_encoder.getPosition() / 383.6 * 0.30159289474462015089241376479483;
//        double left_position = robot.left_encoder.getPosition() / 383.6 * 0.30159289474462015089241376479483;
//
//        odometry.update(
//                imu, right_position, left_position
//        );
//        Pose2d currentRobotPose = odometry.getPoseMeters();


        boolean a = gamepad1.a;
        if (a && !extend) {
            schedule(new SharedCommand(robot, 2, false, outtake));
        }
        extend = a;

        double manual = System.currentTimeMillis();
        telemetry.addData("manual update time ", manual - arm);


        if (intake && robot.bucket.hasFreight()) {
            intake = false;
            schedule(new SharedCommand(robot, 2, false, outtake, done));
        }

        double auto = System.currentTimeMillis();
        telemetry.addData("auto update time ", auto - manual);


        double curr = System.currentTimeMillis();
        telemetry.addData("total loop time", curr - loop);

//        telemetry.addData("imu ", imu.toString());
//        telemetry.addData("left ", left_position);
//        telemetry.addData("right ", right_position);
//        telemetry.addLine(currentRobotPose.toString());
//        telemetry.addData("arm ", robot.arm.pos());
//        telemetry.addData("has freight ", robot.bucket.hasFreight());

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
        telemetry.update();

        loop = System.currentTimeMillis();
    }
}
