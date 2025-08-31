package teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Limelight Homography Testing", group = "A")
public class LimelightTesting extends OpMode {

    private Limelight3A limelight;
    private Servo base, bucket;
    LLResult result;

    private boolean prevA = false;

    //computing with homography
    private static final double[][] H = {
            {2.67590102, 0.619089595, -1004.86983},
            {0.175664699, 0.496868316, 1897.11795},
            {0.000440004881, 0.0146746602, 1.0}
    };
    //computing with trig
    private double[] pixelToFloor(double cx, double cy) {

        double hCamera = 100.0;
        double tiltDownDeg = 30.0;
        double vFOV = 42.0;
        double hFOV = 54.5;
        int imgW = 640;
        int imgH = 480;
        cx = imgH-cx;
        double xOffset = cx - imgW/2.0;
        double yOffset = imgH/2.0 - cy;

        double fV = (imgH/2.0) / Math.tan(Math.toRadians(vFOV/2.0));
        double fH = (imgW/2.0) / Math.tan(Math.toRadians(hFOV/2.0));

        double angleV = Math.toDegrees(Math.atan(yOffset/fV));
        double angleH = Math.toDegrees(Math.atan(xOffset/fH));

        double forward = hCamera / Math.tan(Math.toRadians(tiltDownDeg + angleV));
        double sideways = forward * Math.tan(Math.toRadians(angleH));

        return new double[]{sideways, forward};
    }




    @Override
    public void init() {
        base = hardwareMap.get(Servo.class, "base");
        bucket = hardwareMap.get(Servo.class,"bucket");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
    }

    @Override
    public void start() {
        limelight.start();
        base.setPosition(Values.baseDefault);
        bucket.setPosition(Values.bucketDefault);
    }

    @Override
    public void loop() {
        boolean currA = gamepad1.a;

        if (currA && !prevA) {
            limelightResult();
        }

        prevA = currA;
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private double[] applyHomography(double cx, double cy) {
        double x = H[0][0] * cx + H[0][1] * cy + H[0][2];
        double y = H[1][0] * cx + H[1][1] * cy + H[1][2];
        double w = H[2][0] * cx + H[2][1] * cy + H[2][2];
        if (w == 0) w = 1e-6;
        return new double[]{ x / w, y / w };
    }


    public void limelightResult() {
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
                        bucket.setPosition(Values.bucketGarbage);
                    } else {
                        bucket.setPosition(Values.bucketRecycle);
                    }

                    telemetry.addData("Target",
                            "Conf: %.2f | Class: %s",
                            conf,
                            dr.getClassName());
                    telemetry.addData("Pixel Center", "(%.1f, %.1f)", cx, cy);
                    telemetry.addData("Floor Coords", "(X=%.1f mm, Y=%.1f mm)", sideways, forward);

                    double[] ik = ivk(Math.hypot(forward, sideways), 50);
                    if (ik != null) {
                        telemetry.addData("IK", "Shoulder=%.1f, Elbow=%.1f, Wrist=%.1f",
                                ik[0], ik[1], ik[2]);
                    } else {
                        telemetry.addData("IK", "Out of reach");
                    }
                }
            }
        } else {
            telemetry.addData("Limelight", "No valid result");
        }
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

}
