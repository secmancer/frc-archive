package frc3512.lib.controllers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A base class for subsystem controllers.
 *
 * <p>State, Inputs, and Outputs indices should be specified what they represent in the derived
 * class.
 *
 * @tparam States the number of state estimates in the state vector
 * @tparam Inputs the number of control inputs in the input vector
 * @tparam Outputs the number of local outputs in the output vector
 */
public class ControllerBase<States extends Num, Inputs extends Num, Outputs extends Num> {

  /// Controller reference for current timestep.
  Matrix<States, N1> m_r;

  /// Controller reference for next timestep.
  Matrix<States, N1> m_nextR;

  /// Controller output.
  Matrix<Inputs, N1> m_u;

  ControllerBase() {}

  public Matrix<States, N1> getReferences() {
    return m_r;
  }

  public Matrix<Inputs, N1> getInputs() {
    return m_u;
  }

  public Matrix<Inputs, N1> calculate(Matrix<States, N1> x) {
    return m_u;
  }

  public Matrix<Inputs, N1> calculate(Matrix<States, N1> x, Matrix<States, N1> r) {
    m_nextR = r;
    Matrix<Inputs, N1> u = calculate(x);
    m_r = m_nextR;
    return u;
  }
}
