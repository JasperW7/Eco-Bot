package teleop;

public class Values {
    static final double a0 = 180, a1 = 130, a2 = 130, a3 = 180;
    static final double
            R0 = (double) 40 /45, A0 = (double) 355/255 * a0,
            R1 = (double) 3/4, A1 = (double) 180/76 * a1,
            R2 = (double) 1, A2 = (double) 180/76 * a2,
            R3 = (double) 1, A3 = (double) 355/255 * a3;
    static final double L0 = 169.219, L1 = 185.64, L2 = 200, L3 = 200, L4 = 172.55;
    static final double clawOpen = 0.27, clawClose = 1;
    static final double clawRotAfterDropoff = 0.7;
    static final double baseDefault = 0.5, link1Default = 0, link2Default = 1, clawRotDefault = 0.5,bucketDefault = 0.5;
    static final double link1Dropoff = 0.2, link2Dropoff = 0.33, clawRotDropoff = 0.07;
    static final double bucketRecycle = 0, bucketGarbage = 1;
    static final double baseRecycle = 0.4, baseGarbage = 0.6;

}
