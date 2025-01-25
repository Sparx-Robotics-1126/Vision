// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126.tools;

/** Add your docs here. */
public class MathTools {
    public static int map(int x, int inMin, int inMax, int outMin, int outMax){
        int value = ((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
        return Math.max(outMin, Math.min(outMax, value));
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax){
        double value = (x - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
        return Math.max(outMin, Math.min(outMax, value));
    }

    public static  int clamp(int value, int min, int max){
        return Math.max(min, Math.min(max, value));
    }
}  