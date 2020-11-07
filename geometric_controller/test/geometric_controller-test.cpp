#include "geometric_controller/geometric_controller.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(GeometricControllerTest, acc2quaternion) {
  Eigen::Vector3d acceleration;
  double yaw;
  // Expected result;
  Eigen::Vector4d ref_attitude;
  Eigen::Vector4d attitude;

  // Condition
  acceleration << 0.0, 0.0, 1.0;
  yaw = 0.0;
  // Expected outcome
  ref_attitude << 1.0, 0.0, 0.0, 0.0;

  attitude = geometricCtrl::acc2quaternion(acceleration, yaw);

  ASSERT_TRUE(attitude.isApprox(ref_attitude));

  // Condition
  acceleration << 0.0, 0.0, 1.0;
  yaw = 1.5714;
  // Expected outcome
  ref_attitude << std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw);

  attitude = geometricCtrl::acc2quaternion(acceleration, yaw);

  ASSERT_TRUE(attitude.isApprox(ref_attitude));
}

TEST(GeometricControllerTest, velocityyaw) {
  Eigen::Vector3d vel_x(1.0, 0.0, 0.0);
  double vel_x_yaw = geometricCtrl::getVelocityYaw(vel_x);
  EXPECT_NEAR(0.0, vel_x_yaw, 0.0001);

  Eigen::Vector3d vel_nx(-1.0, 0.0, 0.0);
  double vel_nx_yaw = geometricCtrl::getVelocityYaw(vel_nx);
  EXPECT_NEAR(M_PI, vel_nx_yaw, 0.0001);

  Eigen::Vector3d vel_y(0.0, 1.0, 0.0);
  double vel_y_yaw = geometricCtrl::getVelocityYaw(vel_y);
  EXPECT_NEAR(M_PI_2, vel_y_yaw, 0.0001);

  Eigen::Vector3d vel_ny(0.0, -1.0, 0.0);
  double vel_ny_yaw = geometricCtrl::getVelocityYaw(vel_ny);
  EXPECT_NEAR(-M_PI_2, vel_ny_yaw, 0.0001);
}
