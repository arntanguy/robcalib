#define BOOST_TEST_MODULE TestLinkEye
#include <boost/test/unit_test.hpp>

#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include "robcalib/cost_link_eye.h"
#include "robcalib/calibration.hpp"
#include "robcalib/logging.h"
#include "fixture_park.h"

using namespace robcalib;
using namespace roboptim;
using solver_t = Solver<EigenMatrixDense>;

/**
 * @brief The norm of the cost function residual at the solution should always
 * be zero.
 */
BOOST_AUTO_TEST_CASE(test_cost_at_solution)
{
  LOG(info) << "[CostLinkEyeSE3] Checking whether norm of residual at solution is zero" << std::endl;
  ParkExampleFixture fix;
  auto fd = boost::shared_ptr<CostLinkEyeSE3>(new CostLinkEyeSE3(fix.eye, fix.link));
  auto res1 = (*fd)(fix.expected_se3);
  SumOfC1Squares fSum(fd,
                      "some function");

  auto res = fSum(fix.expected_se3);

  BOOST_REQUIRE_MESSAGE(res[0] < 0.1,
                        "Residual of cost function at expected solution should be zero but is " << res[0]);
}

/**
 * @brief Uses finite differences to check against the analytical jacobian
*  XXX Should test jacobian at several random twists, not just identity
 *
 * @param test_check_jacobian
 */
BOOST_AUTO_TEST_CASE(test_check_jacobian)
{
  LOG(info) << "[CostLinkEyeSE3] Checking jacobian" << std::endl;
  ParkExampleFixture fix;
  CostLinkEyeSE3 cost(fix.eye, fix.link);
  Eigen::Matrix<double, 6, 1> x;
  CostLinkEyeSE3::argument_t start(cost.inputSize());
  start.setZero();
  const auto precision = 10e-3;
  BOOST_REQUIRE_MESSAGE(roboptim::checkJacobian(cost, start, precision), "Finite difference and analytical jacobians for CostLinkEyeSE3 don't match");
}

BOOST_AUTO_TEST_CASE(test_park_example)
{
  LOG(info) << "[CostLinkEyeSE3] Checking optimization on head-eye example from Park and Martin paper" << std::endl;
  ParkExampleFixture fix;

  auto data = std::make_shared<HeadEyeData>();
  data->eye = fix.eye;
  data->link = fix.link;

  robcalib::HeadEyeCalib calib;
  calib.initSolverImpl(data);
  auto res = calib.run();

  std::cout << res << std::endl;
  std::cout << fix.expected_se3 << std::endl;

  BOOST_REQUIRE(res.which() == solver_t::SOLVER_VALUE || solver_t::SOLVER_VALUE_WARNINGS);

  // XXX this test should pass.
  // Make sure this is correct, and adjust the threshold if it is
  if (res.which() == solver_t::SOLVER_VALUE)
  {
    Result& result = boost::get<Result>(res);
    BOOST_REQUIRE_MESSAGE(result.x.matrix().isApprox(fix.expected_se3, 10e-5),
                          "failed with\n result:\n" << result.x.matrix() << "\nexpected:\n" << fix.expected_se3);
  }
  else if (res.which() == solver_t::SOLVER_VALUE_WARNINGS)
  {
    Result& result = boost::get<ResultWithWarnings>(res);
    BOOST_REQUIRE_MESSAGE(result.x.matrix().isApprox(fix.expected_se3, 10e-5),
                          "failed with\n result:\n" << result.x.matrix() << "\nexpected:\n" << fix.expected_se3 << "\nsolver output: " << result);
  }
}
