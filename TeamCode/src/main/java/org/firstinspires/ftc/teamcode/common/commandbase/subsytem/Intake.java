package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;

    public Intake(MotorEx i) {
        intake = i;
    }

    public void start() {
        intake.set(1);
    }

    public void reverse() {
        System.out.println("start start");
        intake.set(-1);
        System.out.println("start end");
    }

    public void stop() {
        intake.set(0);
    }

    public void toggle(){
        if (intake.get()==0){
            start();
        }else if(intake.get()==1){
            reverse();
        }else{
            stop();
        }
    }
}
