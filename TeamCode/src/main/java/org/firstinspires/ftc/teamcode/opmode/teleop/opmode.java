package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.AllianceHubCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.CapCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.SharedCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.AllianceHubPreCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

@TeleOp
public class opmode extends CommandOpMode {
    private Robot robot;
    private BooleanSupplier outtake, cap;
    private Consumer<Boolean> done, done_capping;
    private DoubleSupplier linkage, arm;
    private boolean intake = true;
    private boolean extend;
    private double loop = 0;
    public Alliance alliance = Alliance.BLUE;

    private boolean intakeToggle = false;
    private boolean flag = true;

    private boolean pY = false;
    private boolean pA = false;

    private ElapsedTime timer;

    private boolean alliance_hub = false;
    private boolean middle = false;

    private boolean capping = false;


    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, false);
        outtake = () -> gamepad1.right_bumper;
        done = (a) -> intake = a;
        done_capping = (x) -> capping = !x;
        linkage = () -> gamepad1.right_trigger;
        arm = () -> gamepad1.left_trigger;
        cap = () -> gamepad1.b;

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        if (timer.seconds() > 30 && timer.seconds() < 60 && flag) {
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

        double turn = ether(gamepad1.right_stick_x, 0.685, 0.012, 0.6) / (capping ? 2 : 1);
        double forward = ether(-gamepad1.left_stick_y, 0.685, 0.06, 1) / (capping ? 2.5 : 1);

        robot.drive.arcadeDrive(forward, turn);

        boolean a = gamepad1.a;
        if (a && !extend && !alliance_hub) {
            schedule(new SharedCommand(robot, alliance, outtake, linkage, arm, done));
        } else if (a && !extend && alliance_hub) {
            schedule(new AllianceHubCommand(robot, middle, outtake, done));
        }
        extend = a;


        if (intake && robot.bucket.hasFreight()) {
            intake = false;
            if (!alliance_hub) {
                schedule(new SharedCommand(robot, alliance, outtake, linkage, arm, done));
            } else {
                schedule(new AllianceHubPreCommand(robot).andThen(new AllianceHubCommand(robot, middle, outtake, done)));
            }
        }
        boolean x = gamepad1.x;
        if (x && !intakeToggle) {
            robot.intake.toggle();
        }
        intakeToggle = x;

        boolean y = gamepad1.y;
        if (y && !pY) {
            robot.intake.intake.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake.intake.motorEx.setPower(-0.45);
        } else if (!y && pY) {
            robot.intake.intake.set(0);
        }
        pY = y;

        boolean a_2 = gamepad2.a;
        if (a_2 && !pA) {
            intake = false;
            capping = true;
            schedule(new CapCommand(robot, cap, done, done_capping));
        }
        pA = a_2;

        if (gamepad1.dpad_right) {
            alliance = Alliance.BLUE;
        }

        if (gamepad1.dpad_left) {
            alliance = Alliance.RED;
        }

        if (gamepad1.dpad_up) {
            if (!alliance_hub) {
                alliance_hub = true;
            } else {
                middle = !middle;
            }

        }

        if (gamepad1.dpad_down) {
            alliance_hub = false;
        }


        double time = System.currentTimeMillis();

        robot.currentUpdate(telemetry);
        telemetry.addData("alliance ", alliance.toString());
        telemetry.addData("alliance hub", alliance_hub);
        telemetry.addData("middle", middle);
        telemetry.addData("total loop time", time - loop);
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
        if (x > 0.003) {
            return a_max * (a_min + (1 - a_min)) * v;
        } else if (x < -0.003) {
            return a_max * (-a_min + (1 - a_min)) * v;
        } else {
            return 0;
        }
    }
}
