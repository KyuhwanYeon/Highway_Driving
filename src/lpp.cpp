#include "lpp.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double *Vehicle::state_in(double t)
{
    double s[3];
    double d[3];

    for (int i = 0; i < 3; i++)
    {
        s[i] = start_state[i];
    }
    for (int i = 3; i < 6; i++)
    {
        d[i] = start_state[i];
    }
    static double state[6] = {
        s[0] + (s[1] * t) + s[2] * t * t / 2.0,
        s[1] + s[2] * t,
        s[2],
        d[0] + (d[1] * t) + d[2] * t * t / 2.0,
        d[1] + d[2] * t,
        d[2],
    };
    return state;
}

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T)
{
    /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    MatrixXd A(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;
    // std::cout << A << std::endl;
    MatrixXd B(3, 1);
    B << end[0] - start[0] - start[1] * T - start[2] / 2 * T * T,
        end[1] - start[1] - start[2] * T,
        end[2] - start[2];
    // std::cout << B << std::endl;
    MatrixXd Ai(3, 3);
    Ai = A.inverse();
    MatrixXd Coeff(3, 1);
    Coeff = Ai * B;
    std::cout << Coeff << std::endl;
    vector<double> result = {start[0], start[1], start[2] / 2, Coeff.coeff(0, 0), Coeff.coeff(1, 0), Coeff.coeff(2, 0)};
    return result;
}
