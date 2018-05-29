#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "../src/PID.h"

SCENARIO("PID can control vehicle with PID controll method", "[PID]") {
  GIVEN("A PID object") {
    PID pid;

    WHEN("it's initialized with hyperparameters (0.1, 0.2, 0.3)") {
      pid.Init(0.1, 0.2, 0.3);

      THEN("instance's hyperparameters are (0.1, 0.2, 0.3)") {
        REQUIRE(std::fabs(pid.Kp - 0.1) < 1e-6);
        REQUIRE(std::fabs(pid.Ki - 0.2) < 1e-6);
        REQUIRE(std::fabs(pid.Kd - 0.3) < 1e-6);
      }
    }


    WHEN("UpdateError is called three times") {
      pid.Init(0.1, 0.2, 0.3);
      pid.UpdateError(0.1);
      pid.UpdateError(0.2);
      pid.UpdateError(0.4);

      THEN("P's error is 0.4") {
        REQUIRE(std::fabs(pid.p_error - 0.4) < 1e-6);
      }
      THEN("I's error is 0.7") {
        REQUIRE(std::fabs(pid.i_error - 0.7) < 1e-6);
      }
      THEN("D's error is 0.2") {
        REQUIRE(std::fabs(pid.d_error - 0.2) < 1e-6);
      }
    }


    WHEN("TotalError is called") {
      double total_err = pid.TotalError();

      THEN("total error is -0.24") {
        REQUIRE(std::fabs(total_err + 0.24) < 1e-6);
      }
    }
  }
}
