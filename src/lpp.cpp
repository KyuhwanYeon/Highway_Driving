#include "lpp.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
vector<double> Vehicle::state_in(double t)
{
    vector<double> s(3);
    vector<double> d(3);
    s[0] = start_state[0];
    s[1] = start_state[1];
    s[2] = start_state[2];
    d[0] = start_state[3];
    d[1] = start_state[4];
    d[2] = start_state[5];

    vector<double> state = {
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
vector<double> PTG(vector<double> start_s,
                   vector<double> start_d,
                   vector<double> target_vehicle,
                   vector<double> delta,
                   vector<double> T,
                   vector<double> predictions)
{

// target = predictions[target_vehicle]
//     # generate alternative goals
//     all_goals = []
//     timestep = 0.5
//     t = T - 4 * timestep
//     while t <= T + 4 * timestep:
//         target_state = np.array(target.state_in(t)) + np.array(delta)
//         goal_s = target_state[:3]
//         goal_d = target_state[3:]
//         goals = [(goal_s, goal_d, t)]
//         for _ in range(N_SAMPLES):
//             perturbed = perturb_goal(goal_s, goal_d)
//             goals.append((perturbed[0], perturbed[1], t))
//         all_goals += goals
//         t += timestep
    
//     # find best trajectory
//     trajectories = []
//     for goal in all_goals:
//         s_goal, d_goal, t = goal
//         s_coefficients = JMT(start_s, s_goal, t)
//         d_coefficients = JMT(start_d, d_goal, t)
//         trajectories.append(tuple([s_coefficients, d_coefficients, t]))
    
//     best = min(trajectories, key=lambda tr: calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
//     calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
//     return best

}