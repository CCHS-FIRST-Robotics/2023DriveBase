package frc.robot.subsystems;

/**
 * A time optimal controller for a double integrator.
 * 
 * This is essentially a trapezoid profile parameterized by state instead of by time.
 * 
 * https://underactuated.mit.edu/dp.html#minimum_time_double_integrator
 */
public class TrapezoidProfile {

    Constraints constraints;

    public static class Constraints {

        double maxVelocity;
        double maxAcceleration;

        Constraints() {
            this(0, 0);
        }

        Constraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
    }

    public static class State {

        double position;
        double velocity;

        State() {
            this(0, 0);
        }

        State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }

    /**
     * Constructor for trapezoid profile
     * 
     * @param constraints (Contraints) - the constraints on the profile: max velocity and acceleration
     */
    TrapezoidProfile(Constraints constraints) {
        this.constraints = constraints;
    }

    /**
     * Returns the next acceleration of the double integrator
     * 
     * @param state (State) - the current state of the profile (pos/vel)
     * @param goal (State) - the goal state of the profile (pos/vel)
     * @param dt (double) - timestep duration (seconds)
     */
    public State calculate(State state, State goal, double dt) {
        DenseMatrix x;
        
        double q = state.position;
        double qRef = goal.position;

        double qdot = state.velocity;
        double qdotRef = goal.velocity;

        double uMax = constraints.maxAcceleration;

        // If the position and velocity are within the distance and velocity the
        // controller can travel in one timestep respectively, turn the
        // controller off.
        if (
            Math.abs(qRef - q) <= 0.5 * uMax * dt*dt &&
            Math.abs(qdotRef - qdot) <= uMax * dt
        ) {
            return new State(qRef, qdotRef);
        }

        //TODO: add lin alg library (jeigen) cuz i dont know how lmao https://github.com/hughperkins/jeigen#example-usage-to-multiply-two-matrices
        
        // c₊ = qᵣ − 0.5 / u_max q̇ᵣ²
        // c₋ = qᵣ + 0.5 / u_max q̇ᵣ²
        //
        // if (q̇ < q̇ᵣ and q ≤ 0.5 / u_max q̇² + c₊) or
        //    (q̇ ≥ q̇ᵣ and q < −0.5 / u_max q̇² + c₋)
        //
        // if (q̇ < q̇ᵣ and q ≤ 0.5 / u_max q̇² + qᵣ − 0.5 / u_max q̇ᵣ²) or
        //    (q̇ ≥ q̇ᵣ and q < −0.5 / u_max q̇² + qᵣ + 0.5 / u_max q̇ᵣ²)
        //
        // if (q̇ < q̇ᵣ and q − qᵣ ≤ 0.5 / u_max q̇² − 0.5 / u_max q̇ᵣ²) or
        //    (q̇ ≥ q̇ᵣ and q − qᵣ < −0.5 / u_max q̇² + 0.5 / u_max q̇ᵣ²)
        //
        // if (q̇ < q̇ᵣ and q − qᵣ ≤ 0.5 / u_max q̇² − 0.5 / u_max q̇ᵣ²) or
        //    (q̇ ≥ q̇ᵣ and q − qᵣ < −(0.5 / u_max q̇² − 0.5 / u_max q̇ᵣ²))
        //
        // if (q̇ < q̇ᵣ and q − qᵣ ≤ 0.5 / u_max (q̇² − q̇ᵣ²)) or
        //    (q̇ ≥ q̇ᵣ and q − qᵣ < −0.5 / u_max (q̇² − q̇ᵣ²))
        if (
            (qdot < qdotRef && q - qRef <= 0.5 / uMax * (qdot*qdot - qdotRef*qdotRef)) ||
            (qdot >= qdotRef && q - qRef < -0.5 / uMax * (qdot*qdot - qdotRef*qdotRef))
        ) {
            // Enforce qdot maximum
            if (qdot < constraints.maxVelocity) {
                DenseMatrix x = new DenseMatrix(
                    new double[][] {
                        {q},
                        {qdot},
                        {uMax}
                    }
                );
            } else {
                DenseMatrix x = new DenseMatrix(
                    new double[][] {
                        {q},
                        {qdot},
                        {0}
                    }
                );
            }

        } else {
            // Enforce qdot minimum
            if (qdot > -constraints.maxVelocity) {
                DenseMatrix x = new DenseMatrix(
                    new double[][] {
                        {q},
                        {qdot},
                        {-uMax}
                    }
                );
            } else {
                DenseMatrix x = new DenseMatrix(
                    new double[][] {
                        {q},
                        {qdot},
                        {0}
                    }
                );
            }
        }

        // qₖ₊₁ = qₖ + Tq̇ₖ + 1/2 T²q̈ₖ
        // q̇ₖ₊₁ =       q̇ₖ +      Tq̈ₖ
        DenseMatrx M = new DenseMatrix(
            new double[][] {
                {1, dt, 0.5 * dt*dt},
                {0, 1, dt},
            }
        );
        DenseMatrix xNext = M.mmul(x);
        return TrapezoidProfile.State(xNext.get(0, 0), xNext.get(1, 0));
    }
}