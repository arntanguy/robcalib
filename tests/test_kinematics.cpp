#define BOOST_TEST_MODULE TestKinematics
#include <boost/test/unit_test.hpp>

#include <robcalib/calib_data.h>
#include <robcalib/cost_kinematics_chain.h>
#include <robcalib/logging.h>
#include <robcalib/time_utils.h>
#include <robcalib/calibration.hpp>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/solver.hh>
#include "fixture_kinematics.h"

using namespace robcalib;
using namespace roboptim;

BOOST_AUTO_TEST_CASE(test_hrp4_vrep)
{
  init_log();
  LOG(info) << "[CostKinematicsChain] Checking with hrp4 fixture" << std::endl;

  KinematicsVREPHRP4Fixture fix;

  robcalib::KinematicsEyeCalib calib(fix.estimate_x, fix.known_x);
  calib.createProblem(fix.data);
  auto res = calib.run();

  BOOST_REQUIRE_MESSAGE(res.which() == solver_t::SOLVER_VALUE || solver_t::SOLVER_VALUE_WARNINGS,
                        "[KinematicsEyeCalib] Calibration has not converged as it ought to have.\n"
                            << res);

  // Make sure this is correct, and adjust the threshold if it is
  if (res.which() == solver_t::SOLVER_VALUE)
  {
    Result& result = boost::get<Result>(res);
    BOOST_REQUIRE_MESSAGE(result.x.matrix().isApprox(fix.expected_x, fix.precision), "failed with\n result:\n"
                                                                            << result.x.matrix() << "\nexpected:\n"
                                                                            << fix.expected_x);
  }
  else if (res.which() == solver_t::SOLVER_VALUE_WARNINGS)
  {
    Result& result = boost::get<ResultWithWarnings>(res);
    BOOST_REQUIRE_MESSAGE(result.x.matrix().isApprox(fix.expected_x, fix.precision), "failed with\n result:\n"
                                                                            << result.x.matrix() << "\nexpected:\n"
                                                                            << fix.expected_x
                                                                            << "\nsolver output: " << result);
  }
}
