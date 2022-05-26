package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.SharedCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

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
        robot.arm.loop();
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

        if (intake && robot.bucket.hasFreight()) {
            intake = false;
            schedule(new SharedCommand(robot, 2, false, outtake, done));
        }


        double curr = System.currentTimeMillis();
        telemetry.addData("loop time", curr - loop);

//        telemetry.addData("imu ", imu.toString());
//        telemetry.addData("left ", left_position);
//        telemetry.addData("right ", right_position);
//        telemetry.addLine(currentRobotPose.toString());
//        telemetry.addData("arm ", robot.arm.pos());
//        telemetry.addData("has freight ", robot.bucket.hasFreight());
        telemetry.update();

        loop = System.currentTimeMillis();
    }
}
