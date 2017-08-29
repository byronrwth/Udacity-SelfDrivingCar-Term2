
//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }

  std::cout << "polyeval: for coeffs: " << coeffs << " , x: " << x << " ,result: " << result << std::endl;

  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  std::cout << "polyfit: for xvals: " << xvals << " , yvals: " << yvals << " ,result: " << result << std::endl;
  return result;
}

int main() {
  MPC mpc;
  int iters = 5 ; //50;

  Eigen::VectorXd ptsx(2);
  Eigen::VectorXd ptsy(2);
  ptsx << -100, 100;
  ptsy << -1, -1;

  // TODO: fit a polynomial to the above x and y coordinates
  auto coeffs = polyfit(ptsx, ptsy, 1);

  // NOTE: free feel to play around with these
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  // TODO: calculate the cross track error
  double cte = polyeval(coeffs, x) - y;
  std::cout << "main: cte: " << cte << " , x: " << x << " ,coeffs: " << coeffs << " ,y: " << y << std::endl;


  // TODO: calculate the orientation error
  double epsi = psi - atan(coeffs[1]);
  std::cout << "main: epsi: " << epsi << " , psi: " << psi << " ,coeffs[1]: " << coeffs[1] << " ,atan(coeffs[1]): " << atan(coeffs[1]) << std::endl;



  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    std::cout << "main: before Solve: state: " << state << " ,coeffs: " << coeffs << std::endl;

    auto vars = mpc.Solve(state, coeffs);
    //std::cout << "main: after Solve: vars size: " << vars.size() << " , state: " << state << " ,coeffs: " << coeffs << std::endl;

    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);

    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);
/*    
    std::cout << "x_vals size = " << x_vals.size() << std::endl;
    std::cout << "y_vals size= " << y_vals.size() << std::endl;
    std::cout << "psi_vals size= " << psi_vals.size() << std::endl;
    std::cout << "v_vals size= " << v_vals.size() << std::endl;
    std::cout << "cte_vals size= " << cte_vals.size() << std::endl;
    std::cout << "epsi_vals size= " << epsi_vals.size() << std::endl;
    std::cout << "delta_vals size= " << delta_vals.size() << std::endl;
    std::cout << "a_vals size= " << a_vals.size() << std::endl;
*/
    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];

    std::cout << "------------------------------ " << std::endl;

    std::cout << "main: updated state: " << state << std::endl;

    std::cout << "x = " << vars[0] << std::endl;
    std::cout << "y = " << vars[1] << std::endl;
    std::cout << "psi = " << vars[2] << std::endl;
    std::cout << "v = " << vars[3] << std::endl;
    std::cout << "cte = " << vars[4] << std::endl;
    std::cout << "epsi = " << vars[5] << std::endl;
    std::cout << "delta = " << vars[6] << std::endl;
    std::cout << "a = " << vars[7] << std::endl;
    //std::cout << std::endl;
  }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::subplot(3, 1, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(3, 1, 2);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(3, 1, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::show();
}