package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class Subsystem_Tester extends CommandOpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private GamepadEx gamepad;

        private double time = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void run() {
        super.run();

//        robot.drive.arcadeDrive(-gamepad1.left_stick_y, Math.pow(gamepad1.right_stick_x, 3));
        robot.arm.loop();
//
        Rotation2d imu = new Rotation2d(-robot.imu.getAngularOrientation().firstAngle);
        double right_position = robot.right_encoder.getPosition() / 383.6 * 0.30159289474462015089241376479483;
        double left_position = robot.left_encoder.getPosition() / 383.6 * 0.30159289474462015089241376479483;

        odometry.update(
                imu, right_position, left_position
        );

        Pose2d currentRobotPose = odometry.getPoseMeters();


//        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(()->robot.bucket.close());
//        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->robot.bucket.open());
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> robot.turret.middle());
//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> robot.turret.right());
//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> robot.turret.left());
//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> robot.arm.armIn());
//        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> robot.arm.armShared());
//        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> robot.arm.linkageIn());
//        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> robot.arm.linkageOut());
//
//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> robot.bucket.in());
//        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> robot.bucket.dump());
//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(() -> robot.bucket.rest());
//
//        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> robot.intake.toggle());
//
//
//        telemetry.addData("imu ", imu.toString());
//        telemetry.addData("left ", left_position);
//        telemetry.addData("right ", right_position);
        telemetry.addLine(currentRobotPose.toString());
        telemetry.addData("arm ", robot.arm.pos());

        double curr = System.currentTimeMillis();
        telemetry.addData("time since  last loop", curr - time);
        time = curr;

        telemetry.update();

    }
}
