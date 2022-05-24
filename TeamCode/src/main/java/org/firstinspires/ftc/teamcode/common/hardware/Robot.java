package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Arm;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Bucket;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Intake;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytem.Turret;

public class Robot {
    public DifferentialDrive drive;
    public MotorGroup left;
    public MotorGroup right;
    public Motor.Encoder right_encoder;
    public Motor.Encoder left_encoder;
    public BNO055IMU imu;
    public VoltageSensor batteryVoltageSensor;

    public Turret turret;

    public Arm arm;

    public Bucket bucket;

    public Intake intake;

    public Robot(HardwareMap hardwareMap) {
        MotorEx left_back = new MotorEx(hardwareMap, "lb", Motor.GoBILDA.RPM_435);
        MotorEx left_middle = new MotorEx(hardwareMap, "lm", Motor.GoBILDA.RPM_435);
        MotorEx left_front = new MotorEx(hardwareMap, "lf", Motor.GoBILDA.RPM_435);

        left_back.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        left_middle.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        left_middle.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        left = new MotorGroup(left_back, left_middle, left_front);

        MotorEx right_back = new MotorEx(hardwareMap, "rb", Motor.GoBILDA.RPM_435);
        MotorEx right_middle = new MotorEx(hardwareMap, "rm", Motor.GoBILDA.RPM_435);
        MotorEx right_front = new MotorEx(hardwareMap, "rf", Motor.GoBILDA.RPM_435);

        right_back.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right_middle.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        right_middle.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        right = new MotorGroup(right_back, right_middle, right_front);


        drive = new DifferentialDrive(left, right);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        right_encoder = right_front.encoder;
        right_encoder.setDirection(Motor.Direction.REVERSE);
        left_encoder = left_middle.encoder;

        right_encoder.reset();
        left_encoder.reset();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Servo left = hardwareMap.get(Servo.class, "left");
        Servo right = hardwareMap.get(Servo.class, "right");

        turret = new Turret(left, right);

        DcMotorEx a = hardwareMap.get(DcMotorEx.class, "arm");
        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo l = hardwareMap.get(Servo.class, "linkage");

        arm = new Arm(a, l, batteryVoltageSensor);

        MotorEx i = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake = new Intake(i);

        Servo d = hardwareMap.get(Servo.class, "dump");
        Servo g = hardwareMap.get(Servo.class, "gate");
        DistanceSensor ds = hardwareMap.get(DistanceSensor.class, "distance");
        bucket = new Bucket(d, g, ds);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
