package teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;


@TeleOp(name = "Servo Testing", group = "A")
public class ServoTests extends OpMode {
    private Servo base, link1, link2, rotation, claw,bucket;
    public int servoIdx = 0;
    public ArrayList<String> servos = new ArrayList<>();
    public ArrayList<Double> servoVals = new ArrayList<Double>();
    public double basePos=0.5, link1Pos=0.5, link2Pos=0.5, rotationPos=0.5, clawPos=0.5,bucketPos = 0.5;
    public boolean xPrev, bPrev, aPrev, yPrev;

    /** This method is call once when init is played **/
    @Override
    public void init() {
        base = hardwareMap.get(Servo.class,"base");
        link1 = hardwareMap.get(Servo.class,"linkage1");
        link2 = hardwareMap.get(Servo.class,"linkage2");
        rotation = hardwareMap.get(Servo.class,"rotation");
        bucket = hardwareMap.get(Servo.class,"bucket");
        claw = hardwareMap.get(Servo.class,"claw");
        servos.add("Base"); servos.add("Linkage 1"); servos.add("Linkage 2"); servos.add("Rotation"); servos.add("Claw");servos.add("Bucket");
        servoVals.add(basePos); servoVals.add(link1Pos); servoVals.add(link2Pos); servoVals.add(rotationPos); servoVals.add(clawPos);servoVals.add(bucketPos);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        boolean xCurr = gamepad1.x;
        if (xCurr && !xPrev){
            servoIdx = (servoIdx - 1 + servos.size()) % servos.size();
        }
        xPrev = xCurr;

        boolean bCurr = gamepad1.b;
        if (bCurr && !bPrev){
            servoIdx = (servoIdx + 1) % servos.size();
        }
        bPrev = bCurr;

        boolean yCurr = gamepad1.y;
        if (yCurr && !yPrev){
            servoVals.set(servoIdx, Math.min(1.0, servoVals.get(servoIdx) + 0.01));
        }
        yPrev = yCurr;

        boolean aCurr = gamepad1.a;
        if (aCurr && !aPrev){
            servoVals.set(servoIdx, Math.max(0.0, servoVals.get(servoIdx) - 0.01));
        }
        aPrev = aCurr;



        telemetry.addData("Current Servo: ", servos.get(servoIdx));
        telemetry.addData("Current Value: ", servoVals.get(servoIdx));
        turret(servoVals.get(0),servoVals.get(1),servoVals.get(2), servoVals.get(3),servoVals.get(4));
        bucket.setPosition(servoVals.get(5));
        telemetry.update();
    }


    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }

    public void turret(double basePos,double link1Pos, double link2Pos, double rotationPos, double clawPos){
        base.setPosition(basePos);
        link1.setPosition(link1Pos);
        link2.setPosition(link2Pos);
        rotation.setPosition(rotationPos);
        claw.setPosition(clawPos);
    }
}