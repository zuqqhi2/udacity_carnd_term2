#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "../src/helper_functions.h"
#include "../src/particle_filter.h"

SCENARIO("ParticleFilter can be initialized and predicted", "[ParticleFilter]") {
  GIVEN("A ParticleFilter initialized with position (0.0, 0.0, 0.0)") {
    double         sigma_pos[3] = {0.01, 0.01, 0.01};
    ParticleFilter pf;
    pf.init(0.0, 0.0, 0.0, sigma_pos);

    WHEN("no additional action is done") {
      ParticleFilter pf;
      pf.init(0.0, 0.0, 0.0, sigma_pos);

      THEN("num of particles are 100") { REQUIRE(pf.particles.size() == 100); }

      THEN("each particles have different position") {
        REQUIRE(fabs(pf.particles[0].x - pf.particles[1].x) > 0);
        REQUIRE(fabs(pf.particles[0].y - pf.particles[1].y) > 0);
        REQUIRE(fabs(pf.particles[0].theta - pf.particles[1].theta) > 0);
      }

      THEN("particle's position are decided by normal distribution") {
        double ave_x     = 0.0;
        double ave_y     = 0.0;
        double ave_theta = 0.0;
        for (unsigned int i = 0; i < pf.particles.size(); i++) {
          ave_x += pf.particles[i].x;
          ave_y += pf.particles[i].y;
          ave_theta += pf.particles[i].theta;
        }
        ave_x /= pf.particles.size();
        ave_y /= pf.particles.size();
        ave_theta /= pf.particles.size();

        REQUIRE(fabs(ave_x) < 1.0);
        REQUIRE(fabs(ave_y) < 1.0);
        REQUIRE(fabs(ave_theta) < 1.0);
      }

      THEN("status becomes initialized") {
        REQUIRE(pf.initialized());
      }
    }


    WHEN("prediction step is executed") {
      pf.prediction(0.1, sigma_pos, 1.0, 0.01);

      THEN("particles moved by motion model and normal distribution noise") {
        double ave_x     = 0.0;
        double ave_y     = 0.0;
        double ave_theta = 0.0;
        for (unsigned int i = 0; i < pf.particles.size(); i++) {
          ave_x += pf.particles[i].x;
          ave_y += pf.particles[i].y;
          ave_theta += pf.particles[i].theta;
        }
        ave_x /= pf.particles.size();
        ave_y /= pf.particles.size();
        ave_theta /= pf.particles.size();

        REQUIRE(fabs(ave_x) < 1.0);
        REQUIRE(fabs(ave_y) < 1.0);
        REQUIRE(fabs(ave_theta) < 1.0);
      }
    }


    WHEN("dataAssociation step is executed") {
      std::vector<LandmarkObs> predicted;
      LandmarkObs              p1 = {0, 0.0, 0.0};
      LandmarkObs              p2 = {1, 5.0, 0.0};
      LandmarkObs              p3 = {2, 5.0, 5.0};
      LandmarkObs              p4 = {3, 0.0, 5.0};
      predicted.push_back(p1);
      predicted.push_back(p2);
      predicted.push_back(p3);
      predicted.push_back(p4);

      std::vector<LandmarkObs> observations;
      LandmarkObs              o1 = {-1, 1.0, 1.0};
      LandmarkObs              o2 = {-1, 6.0, -1.0};
      LandmarkObs              o3 = {-1, 6.5, 3.5};
      LandmarkObs              o4 = {-1, 1.5, 5.0};
      LandmarkObs              o5 = {-1, -100.0, -100.0};
      observations.push_back(o1);
      observations.push_back(o2);
      observations.push_back(o3);
      observations.push_back(o4);
      observations.push_back(o5);

      pf.dataAssociation(predicted, observations);

      THEN("observed landmarks are assigned to closest ground truth landmarks") {
        REQUIRE(observations[0].id == 0);
        REQUIRE(observations[1].id == 1);
        REQUIRE(observations[2].id == 2);
        REQUIRE(observations[3].id == 3);
        REQUIRE(observations[4].id == 0);
      }
    }


    WHEN("resamping") {
      pf.resample();

      THEN("num of particles are same as before") {
        REQUIRE(pf.particles.size() == 100);
      }
    }
  }
}
