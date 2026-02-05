package org.firstinspires.ftc.teamcode.util;

public class Misc {
    public static <T extends Number> double Clamp(T value,T min, T max) {
        double Value = (double) value;
        double Min = (double) min;
        double Max = (double) max;
        return Math.min(Math.max(Value,Min),Max);
    }
    public static <T extends Number> double Map(T value, T valueMin, T valueMax, T wantMin, T wantMax){
        return wantMin+(value-valueMin)*((wantMax-wantMin)/(valueMax-valueMin));
    }
}
