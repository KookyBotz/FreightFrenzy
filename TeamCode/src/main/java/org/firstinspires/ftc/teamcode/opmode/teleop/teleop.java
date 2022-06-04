package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private DoubleSupplier linkage;
    private boolean intake = true;
    private boolean extend;
    private double loop = 0;
    public Alliance alliance = Alliance.BLUE;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        outtake = () -> gamepad1.right_bumper;
        done = (a) -> intake = a;
        linkage = () -> gamepad1.right_trigger;

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
        robot.intake.start();
    }

    @Override
    public void run() {
        super.run();

        robot.drive.arcadeDrive(
                -gamepad1.left_stick_y,
                scale(gamepad1.right_stick_x, 0.6)
        );


        robot.arm.loop();


        boolean a = gamepad1.a;
        if (a && !extend) {
            schedule(new SharedCommand(robot, alliance, outtake, done));
        }
        extend = a;


        if (intake && robot.bucket.hasFreight()) {
            intake = false;
            schedule(new SharedCommand(robot, alliance, outtake, linkage, done));
        }


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
        telemetry.update();

        loop = System.currentTimeMillis();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

}
