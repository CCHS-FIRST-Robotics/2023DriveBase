package frc.robot.utils;

public class R2Vector {
    double x, y;

    public R2Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public R2Vector() {
        this(0, 0);
    }

    public double mag() {
        return Math.sqrt(x*x + y*y);
    }

    public double dir() {
        return Math.atan2(y, x);
    }

    public R2Vector add(R2Vector vector) {
        return new R2Vector(x + vector.x, y + vector.y);
    }

    public R2Vector sub(R2Vector vector) {
        return new R2Vector(x - vector.x, y - vector.y);
    }

    public R2Vector multiply(double factor) {
        return new R2Vector(x * factor, y * factor);
    }

    public R2Vector pow(double exp) {
        return new R2Vector(Math.pow(x, exp), Math.pow(y, exp));
    }
}