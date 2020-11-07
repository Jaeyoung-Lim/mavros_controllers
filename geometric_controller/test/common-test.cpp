#include "geometric_controller/common.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(CommonTest, toEigenFromPoint) {
  Eigen::Vector3d test_vector, compare_vector;
  test_vector << 1.0, 2.0, 3.0;

  geometry_msgs::Point p;
  p.x = test_vector[0];
  p.y = test_vector[1];
  p.z = test_vector[2];
  compare_vector = toEigen(p);

  ASSERT_TRUE(compare_vector.isApprox(test_vector));
}

TEST(CommonTest, toEigenFromVector3) {
  Eigen::Vector3d test_vector, compare_vector;
  test_vector << 1.0, 2.0, 3.0;

  geometry_msgs::Vector3 p;
  p.x = test_vector[0];
  p.y = test_vector[1];
  p.z = test_vector[2];
  compare_vector = toEigen(p);

  ASSERT_TRUE(compare_vector.isApprox(test_vector));
}
