package frc.robot.utils;

public class Vector {
    double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double mag() {
        return Math.sqrt(x*x + y*y);
    }

    public double dir() {
        return Math.atan2(y, x);
    }

    public Vector add(Vector vector) {
        return new Vector(x + vector.x, y + vector.y);
    }

    public Vector sub(Vector vector) {
        return new Vector(x - vector.x, y - vector.y);
    }

    public Vector multiply(double factor) {
        return new Vector(x * factor, y * factor);
    }
}