package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

public class TrigMath {
    public static <T extends Number> double subtractAngles(T a1, T a2) {
        double A1=(double)a1;
        double A2=(double)a2;
        double X=cos(A1)*cos(A2)+sin(A1)*sin(A2);
        double Y=sin(A1)*cos(A2)-cos(A1)*sin(A2);
        return atan2(Y,X);
    }
}
