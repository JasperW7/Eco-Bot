package teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Eco Bot Auto", group = "A")
public class EcoBotAuto extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    LLResult result;
    private Timer timer;
    private Servo base, link1, link2, rotation, claw, bucket;

    private double basePos = Values.baseDefault, link1Pos = Values.link1Default, link2Pos = Values.link2Default, rotationPos = Values.clawRotDefault, clawPos = Values.clawClose, bucketPos = Values.bucketDefault;
    private final Pose startPose = new Pose(0,0,0);

    private boolean init = true;
    private boolean clawOpen = false;
    boolean rightBumperPrevState = false, leftBumperPrevState = false;

    private int turretState = 0;

    private double targetBase, targetLink1, targetLink2, targetRotation, targetClaw, targetBucket;
    private double currX, currY;

    private double link1Step = 0.002, link2Step = 0.002, rotationStep = 0.01,clawStep = 0.01;
    private static final double[][] H = {
            {2.67590102, 0.619089595, -1004.86983},
            {0.175664699, 0.496868316, 1897.11795},
            {0.000440004881, 0.0146746602, 1.0}
    };

    public enum Mode {
        REST,
        INTAKING,
        INTAKING_GRAB,
        DROPOFF
    }
    Mode mode = Mode.REST;

    private enum Bin { RECYCLE, GARBAGE }
    private Bin lastBin = Bin.RECYCLE;
    private double[] lastIk = null;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        base = hardwareMap.get(Servo.class,"base");
        link1 = hardwareMap.get(Servo.class,"linkage1");
        link2 = hardwareMap.get(Servo.class,"linkage2");
        rotation = hardwareMap.get(Servo.class,"rotation");
        claw = hardwareMap.get(Servo.class,"claw");
        bucket = hardwareMap.get(Servo.class,"bucket");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(7);

        follower.setStartingPose(startPose);
        timer = new Timer();
    }

    @Override
    public void init_loop() {
        turretDefault();
    }

    @Override
    public void start() {
        if (limelight != null) {
            limelight.start();
        }
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        boolean rightBumperCurrentState = gamepad1.right_bumper;
        if (rightBumperCurrentState && !rightBumperPrevState) {
            if (mode == Mode.DROPOFF && turretState== 0) {
                mode = Mode.REST;
                init = true;
            } else if (mode == Mode.INTAKING && turretState== 0) {
                mode = Mode.REST;
                init = true;
            } else if (mode == Mode.INTAKING_GRAB && turretState ==0){
                mode = Mode.REST;
                init = true;
            }
        }
        rightBumperPrevState = rightBumperCurrentState;

        boolean leftBumperCurrentState = gamepad1.left_bumper;
        if (leftBumperCurrentState && !leftBumperPrevState) {
            if (mode == Mode.REST && turretState== 0) {
                mode = Mode.INTAKING;
                init = true;
            }else if (mode == Mode.INTAKING && turretState == 0){
                mode = Mode.INTAKING_GRAB;
                init = true;
            } else if (mode == Mode.INTAKING_GRAB && turretState==0){
                mode = Mode.DROPOFF;
                init = true;
            }
        }
        leftBumperPrevState = leftBumperCurrentState;

        switch(mode){
            case REST:
                if (init) {
                    turretDefault();
                    init = false;
                }
                break;

            case INTAKING:
                if (init){
                    double[]cameraResult = limelightResult();
                    if (cameraResult==null){
                        telemetry.addLine("INTAKING: waiting for Limelight...");
                        break;
                    }

                    currX = cameraResult[1]; currY = cameraResult[0];
                    targetBase = degreesToServo(Math.toDegrees(Math.atan2(cameraResult[1],cameraResult[0])),Values.R0,Values.A0);
                    targetLink1 = degreesToServo(cameraResult[2],Values.R1,Values.A1);
                    targetLink2 = degreesToServo(cameraResult[3],Values.R2,Values.A2);
                    targetRotation = degreesToServo(cameraResult[4],Values.R3,Values.A3);
                    targetClaw = Values.clawOpen;

                    setState(1);
                    init = false;
                }
                break;

            case INTAKING_GRAB:
                if (init){
                    double[] ik = ivk(Math.hypot(currX,currY),80);
                    if (ik == null && lastIk != null) ik = lastIk;
                    if (ik != null) {
                        targetLink1 = degreesToServo(ik[0],Values.R1,Values.A1);
                        targetLink2 = degreesToServo(ik[1],Values.R2,Values.A2);
                        targetRotation = degreesToServo(ik[2],Values.R3,Values.A3);
                        targetClaw = Values.clawClose;
                        setState(1);
                        init = false;
                    } else {
                        telemetry.addLine("INTAKING_GRAB: no IK available yet");
                    }
                }
                break;

            case DROPOFF:
                if (init){
                    turretDropoff();
                    init = false;
                }
                break;
        }

        updateTurret();

        base.setPosition(basePos);
        link1.setPosition(link1Pos);
        link2.setPosition(link2Pos);
        rotation.setPosition(rotationPos);
        claw.setPosition(clawPos);
        bucket.setPosition(bucketPos);

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y/5,
                -gamepad1.left_stick_x/5,
                -gamepad1.right_stick_x/5,
                true
        );
        follower.update();

        telemetry.addData("Mode", mode);
        telemetry.addData("turretState", turretState);
        telemetry.addData("Targets","base=%.2f l1=%.2f l2=%.2f rot=%.2f claw=%.2f bucket=%.2f",
                targetBase,targetLink1,targetLink2,targetRotation,targetClaw,targetBucket);
        telemetry.addData("Positions","base=%.2f l1=%.2f l2=%.2f rot=%.2f claw=%.2f bucket=%.2f",
                basePos,link1Pos,link2Pos,rotationPos,clawPos,bucketPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public void turretDefault() {
        if (turretState != 0) return;

        targetBase     = Values.baseDefault;
        targetLink1    = Values.link1Default;
        targetLink2    = Values.link2Default;
        targetRotation = Values.clawRotDefault;
        targetClaw     = Values.clawClose;
        targetBucket   = Values.bucketDefault;

        setState(1);
    }

    public void turretDropoff(){
        if (turretState!=0) return;

        if (lastBin == Bin.GARBAGE) {
            targetBase   = Values.baseGarbage;
            targetBucket = Values.bucketGarbage;
        } else {
            targetBase   = Values.baseRecycle;
            targetBucket = Values.bucketRecycle;
        }

        targetLink1    = Values.link1Dropoff;
        targetLink2    = Values.link2Dropoff;
        targetRotation = Values.clawRotDropoff;
        targetClaw     = Values.clawOpen;

        setState(1);
    }

    public void updateTurret() {
//        switch (turretState) {
//            case 1:
//                basePos = targetBase;
//                bucketPos = targetBucket;
//                setState(2);
//                break;
//            case 2:
//                if (timer.getElapsedTimeSeconds() > 2) {
//                    rotationPos = targetRotation;
//                    setState(3);
//                }
//                break;
//            case 3:
//                if (timer.getElapsedTimeSeconds()>0.5){
//                    if (Math.abs(link2Pos - targetLink2) > link2Step) {
//                        if (link2Pos < targetLink2) link2Pos += link2Step;
//                        else link2Pos -= link2Step;
//                    } else {
//                        link2Pos = targetLink2;
//                        setState(4);
//                    }
//                }
//                break;
//            case 4:
//                if (timer.getElapsedTimeSeconds() > 2) {
//                    if (Math.abs(link1Pos - targetLink1) > link1Step) {
//                        if (link1Pos < targetLink1) link1Pos += link1Step;
//                        else link1Pos -= link1Step;
//                    } else {
//                        link1Pos = targetLink1;
//                        setState(5);
//                    }
//                }
//                break;
//            case 5:
//                if (timer.getElapsedTimeSeconds()>1){
//                    clawPos = targetClaw;
//                    setState(0);
//                }
//                break;
//        }
        switch(turretState){
            case 1:

                boolean link1Done = false;
                boolean link2Done = false;
                boolean link3Done = false;

                if (Math.abs(link1Pos - targetLink1) > link1Step) {
                    if (link1Pos < targetLink1) link1Pos += link1Step;
                    else link1Pos -= link1Step;
                } else {
                    link1Pos = targetLink1;
                    link1Done = true;
                }

                if (Math.abs(link2Pos - targetLink2) > link2Step) {
                    if (link2Pos < targetLink2) link2Pos += link2Step;
                    else link2Pos -= link2Step;
                } else {
                    link2Pos = targetLink2;
                    link2Done = true;
                }

                if (Math.abs(rotationPos - Values.clawRotAfterDropoff) > rotationStep) {
                    if (rotationPos < Values.clawRotAfterDropoff) rotationPos += rotationStep;
                    else rotationPos -= rotationStep;
                } else {
                    rotationPos = Values.clawRotAfterDropoff;
                    link3Done = true;
                }

                if (link1Done && link2Done && link3Done) {
                    setState(2);
                }
                break;

            case 2:
                bucketPos = targetBucket;
                basePos = targetBase;
                rotationPos = targetRotation;
                setState(3);
                break;
            case 3:
                if (timer.getElapsedTimeSeconds()>1){
                    if (Math.abs(clawPos - targetClaw) > clawStep) {
                        if (clawPos < targetClaw) clawPos += clawStep;
                        else clawPos -= clawStep;
                    } else {
                        clawPos = targetClaw;
                        setState(0);
                    }
                }
                break;


        }


    }

    public void setState(int nextState) {
        turretState = nextState;
        timer.resetTimer();
    }

    public double degreesToServo(double degree, double gearRatio, double servoAngle) {
        double servoVal = -degree / (gearRatio * servoAngle) + 0.5;
        return Math.max(0.0, Math.min(1.0, servoVal));
    }

    private double[] applyHomography(double cx, double cy) {
        double x = H[0][0] * cx + H[0][1] * cy + H[0][2];
        double y = H[1][0] * cx + H[1][1] * cy + H[1][2];
        double w = H[2][0] * cx + H[2][1] * cy + H[2][2];
        if (w == 0) w = 1e-6;
        return new double[]{ x / w, y / w };
    }

    public double[] ivk(double tipX, double tipY) {
        double L0 = Values.L0;
        double L1 = Values.L1;
        double L2 = Values.L2;
        double L3 = Values.L3;
        double L4 = Values.L4;

        double theta0 = Math.toRadians(60);

        double wristX = tipX;
        double wristY = tipY + L4;

        double dx = wristX - L1 * Math.cos(theta0);
        double dy = wristY - L0 - L1 * Math.sin(theta0);
        double D = Math.hypot(dx, dy);

        double minReach = 100, maxReach = 450;
        if (D < minReach || D > maxReach) return null;

        double theta1 = Math.atan2(dy, dx) + Math.acos((L2*L2 + D*D - L3*L3) / (2*L2*D));
        double theta2 = theta1 - (Math.PI - Math.acos((L2*L2 + L3*L3 - D*D) / (2*L2*L3)));
        double theta3 = Math.toRadians(150) + theta1 + theta2;

        double shoulder_rel = Math.toDegrees(theta1 - theta0);
        double elbow_rel = Math.toDegrees(theta2 - theta1);
        double wrist_rel = -150 - shoulder_rel - elbow_rel;

        return new double[]{shoulder_rel, elbow_rel, wrist_rel};
    }

    public double[] limelightResult() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            if (!detectorResults.isEmpty()) {
                detectorResults.sort((a, b) -> Double.compare(b.getConfidence(), a.getConfidence()));
                LLResultTypes.DetectorResult dr = detectorResults.get(0);

                double conf = dr.getConfidence();
                List<List<Double>> corners = dr.getTargetCorners();
                if (corners.size() == 4) {
                    double cx = 0, cy = 0;
                    for (List<Double> c : corners) {
                        cx += c.get(0);
                        cy += c.get(1);
                    }
                    cx /= 4.0;
                    cy /= 4.0;

                    double[] floorCoords = applyHomography(cx, cy);
                    double sideways = floorCoords[0];
                    double forward = floorCoords[1];

                    if (Objects.equals(dr.getClassName(), "plastic")) {
                        targetBucket = Values.bucketGarbage;
                        lastBin = Bin.GARBAGE;
                    } else {
                        targetBucket = Values.bucketRecycle;
                        lastBin = Bin.RECYCLE;
                    }

                    telemetry.addData("Target","Conf: %.2f | Class: %s", conf, dr.getClassName());
                    telemetry.addData("Pixel Center", "(%.1f, %.1f)", cx, cy);
                    telemetry.addData("Floor Coords", "(X=%.1f mm, Y=%.1f mm)", sideways, forward);

                    double[] ik = ivk(Math.hypot(forward, sideways), 250);
                    if (ik != null) {
                        lastIk = ik;
                        telemetry.addData("IK", "Shoulder=%.1f, Elbow=%.1f, Wrist=%.1f", ik[0], ik[1], ik[2]);
                        return new double[] {forward,sideways, ik[0],ik[1],ik[2]};
                    } else {
                        telemetry.addData("IK", "Out of reach");
                    }
                }
            }
        } else {
            telemetry.addData("Limelight", "No valid result");
        }
        return null;
    }
}
