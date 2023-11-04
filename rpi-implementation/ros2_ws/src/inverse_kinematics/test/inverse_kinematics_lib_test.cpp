/**
 * inverse_kinematics_test.cpp
*/


#include "../include/inverse_kinematics.hpp"
#include <stdio.h>
#include <string>


float float_tolerance = 0.002f;


bool FloatIsClose(float a, float b);
int TestCircleIntersection(float c1_x, float c1_y, float r1,
                           float c2_x, float c2_y, float r2,
                           int num_intersections_true,
                           float x_ccw_true, float y_ccw_true,
                           float x_cw_true, float y_cw_true);
int TestInverseKinematics(
    float x, float y, float z,
    float joint_1_plane_offset, float linkage_2_length, float linkage_3_length,
    float j1_config, float j2_config,
    int reachable_true,
    float j1_angle_true, float j2_angle_true, float j3_angle_true);


int main(int argc, char* argv[]) {
  float c1_x, c1_y, r1;
  float c2_x, c2_y, r2;
  int num_intersections_true;
  float x_ccw_true, y_ccw_true;
  float x_cw_true, y_cw_true;
  float x, y, z;
  float joint_1_plane_offset, linkage_2_length, linkage_3_length;
  float j1_config, j2_config;
  int reachable_true;
  float j1_angle_true, j2_angle_true, j3_angle_true;
  int test_result;
  int num_succeeded;
  int num_failed;

  if (argc > 1) {
    float_tolerance = std::stof(argv[1]);
  }

  printf("Testing CircleIntersection with tolerance: %f\n", float_tolerance);
  num_succeeded = 0;
  num_failed = 0;

  // Test 1
  c1_x = 0.0f;
  c1_y = 0.0f;
  r1 = 1.0f;
  c2_x = 2.0f;
  c2_y = 0.0f;
  r2 = 1.0f;
  num_intersections_true = 1;
  x_ccw_true = 1.0f;
  y_ccw_true = 0.0f;
  x_cw_true = 1.0f;
  y_cw_true = 0.0f;
  test_result = TestCircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2,
                                       num_intersections_true,
                                       x_ccw_true, y_ccw_true,
                                       x_cw_true, y_cw_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 1: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 1: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect ccw num_intersections\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect ccw x\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect ccw y\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect cw num_intersections\n");
    }
    if (test_result & (1 << 4)) {
      printf("  incorrect cw x\n");
    }
    if (test_result & (1 << 5)) {
      printf("  incorrect cw y\n");
    }
  }

  // Test 2
  c1_x = 0.0f;
  c1_y = 0.0f;
  r1 = 1.0f;
  c2_x = 1.99f;
  c2_y = 0.0f;
  r2 = 1.0f;
  num_intersections_true = 2;
  x_ccw_true = 0.995f;
  y_ccw_true = -0.0998749217772f;
  x_cw_true = 0.995f;
  y_cw_true = 0.0998749217772f;
  test_result = TestCircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2,
                                       num_intersections_true,
                                       x_ccw_true, y_ccw_true,
                                       x_cw_true, y_cw_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 2: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 2: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect ccw num_intersections\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect ccw x\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect ccw y\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect cw num_intersections\n");
    }
    if (test_result & (1 << 4)) {
      printf("  incorrect cw x\n");
    }
    if (test_result & (1 << 5)) {
      printf("  incorrect cw y\n");
    }
  }

  // Test 3
  c1_x = -0.36f;
  c1_y = 0.28f;
  r1 = 0.29f;
  c2_x = 0.13f;
  c2_y = -0.37f;
  r2 = 1.0f;
  num_intersections_true = 2;
  x_ccw_true = -0.649080434006f;
  y_ccw_true = 0.256923980519f;
  x_cw_true = -0.258237706652f;
  y_cw_true = 0.551559267293f;
  test_result = TestCircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2,
                                       num_intersections_true,
                                       x_ccw_true, y_ccw_true,
                                       x_cw_true, y_cw_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 3: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 3: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect ccw num_intersections\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect ccw x\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect ccw y\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect cw num_intersections\n");
    }
    if (test_result & (1 << 4)) {
      printf("  incorrect cw x\n");
    }
    if (test_result & (1 << 5)) {
      printf("  incorrect cw y\n");
    }
  }

  // Test 4
  c1_x = -0.36f;
  c1_y = 0.28f;
  r1 = 0.29f;
  c2_x = 0.75f;
  c2_y = -0.54f;
  r2 = 1.0f;
  num_intersections_true = 0;
  x_ccw_true = 0.0f;
  y_ccw_true = 0.0f;
  x_cw_true = 0.0f;
  y_cw_true = 0.0f;
  test_result = TestCircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2,
                                       num_intersections_true,
                                       x_ccw_true, y_ccw_true,
                                       x_cw_true, y_cw_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 4: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 4: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect ccw num_intersections\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect ccw x\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect ccw y\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect cw num_intersections\n");
    }
    if (test_result & (1 << 4)) {
      printf("  incorrect cw x\n");
    }
    if (test_result & (1 << 5)) {
      printf("  incorrect cw y\n");
    }
  }

  // Test 5
  c1_x = 0.32f;
  c1_y = -0.98f;
  r1 = 0.29f;
  c2_x = 0.75f;
  c2_y = -0.54f;
  r2 = 1.0f;
  num_intersections_true = 0;
  x_ccw_true = 0.0f;
  y_ccw_true = 0.0f;
  x_cw_true = 0.0f;
  y_cw_true = 0.0f;
  test_result = TestCircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2,
                                       num_intersections_true,
                                       x_ccw_true, y_ccw_true,
                                       x_cw_true, y_cw_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 5: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 5: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect ccw num_intersections\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect ccw x\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect ccw y\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect cw num_intersections\n");
    }
    if (test_result & (1 << 4)) {
      printf("  incorrect cw x\n");
    }
    if (test_result & (1 << 5)) {
      printf("  incorrect cw y\n");
    }
  }

  printf("Finished testing CircleIntersection.\n");
  printf("  SUCCEEDED: %d\n", num_succeeded);
  printf("  FAILED:    %d\n", num_failed);
  printf("\n");

  printf("Testing InverseKinematics with tolerance: %f\n", float_tolerance);
  num_succeeded = 0;
  num_failed = 0;

  // Test 1
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.5f;
  linkage_3_length = 0.5f;
  j1_config = -1.0f;
  j2_config = -1.0f;
  reachable_true = 0;
  j1_angle_true = 0.0f;
  j2_angle_true = 0.0f;
  j3_angle_true = 0.0f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 1: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 1: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 2
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.5f;
  linkage_3_length = 0.5f;
  j1_config = -1.0f;
  j2_config = 1.0f;
  reachable_true = 0;
  j1_angle_true = 0.0f;
  j2_angle_true = 0.0f;
  j3_angle_true = 0.0f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 2: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 2: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 3
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.75f;
  linkage_3_length = 0.75f;
  j1_config = -1.0f;
  j2_config = -1.0f;
  reachable_true = 0;
  j1_angle_true = 0.0f;
  j2_angle_true = 0.841068670568f;
  j3_angle_true = -1.68213734114f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 3: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 3: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 4
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.75f;
  linkage_3_length = 0.75f;
  j1_config = -1.0f;
  j2_config = 1.0f;
  reachable_true = 0;
  j1_angle_true = 0.0f;
  j2_angle_true = -0.841068670568f;
  j3_angle_true = 1.68213734114f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 4: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 4: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 5
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.51f;
  linkage_3_length = 0.5f;
  j1_config = 1.0f;
  j2_config = -1.0f;
  reachable_true = 0;
  j1_angle_true = -0.927295218002f;
  j2_angle_true = 0.139439046248f;
  j3_angle_true = -0.281685646257f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 5: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 5: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 6
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.51f;
  linkage_3_length = 0.5f;
  j1_config = 1.0f;
  j2_config = 1.0f;
  reachable_true = 0;
  j1_angle_true = -0.927295218002f;
  j2_angle_true = -0.139439046248f;
  j3_angle_true = 0.281685646257f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 6: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 6: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 7
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.75f;
  linkage_3_length = 0.75f;
  j1_config = 1.0f;
  j2_config = -1.0f;
  reachable_true = 0;
  j1_angle_true = -0.927295218002f;
  j2_angle_true = 0.841068670568f;
  j3_angle_true = -1.68213734114f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 7: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 7: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  // Test 8
  x = 0.5f;
  y = 0.0f;
  z = -1.0f;
  joint_1_plane_offset = 0.5f;
  linkage_2_length = 0.75f;
  linkage_3_length = 0.75f;
  j1_config = 1.0f;
  j2_config = 1.0f;
  reachable_true = 0;
  j1_angle_true = -0.927295218002f;
  j2_angle_true = -0.841068670568f;
  j3_angle_true = 1.68213734114f;
  test_result = TestInverseKinematics(x, y, z,
                                      joint_1_plane_offset,
                                      linkage_2_length,
                                      linkage_3_length,
                                      j1_config, j2_config,
                                      reachable_true,
                                      j1_angle_true,
                                      j2_angle_true,
                                      j3_angle_true);
  if (test_result == 0) {
    num_succeeded += 1;
    printf("Test 8: succeeded\n");
  } else {
    num_failed += 1;
    printf("Test 8: FAILED\n");
    if (test_result & (1 << 0)) {
      printf("  incorrect reachability report\n");
    }
    if (test_result & (1 << 1)) {
      printf("  incorrect J1 angle\n");
    }
    if (test_result & (1 << 2)) {
      printf("  incorrect J2 angle\n");
    }
    if (test_result & (1 << 3)) {
      printf("  incorrect J3 angle\n");
    }
  }

  printf("Finished testing InverseKinematics.\n");
  printf("  SUCCEEDED: %d\n", num_succeeded);
  printf("  FAILED:    %d\n", num_failed);
  printf("\n");

  printf("All tests completed. Exiting.\n");
}


bool FloatIsClose(float a, float b) {
  return ((a > b) ? (a - b) : (b - a)) <= float_tolerance;
}


int TestCircleIntersection(float c1_x, float c1_y, float r1,
                           float c2_x, float c2_y, float r2,
                           int num_intersections_true,
                           float x_ccw_true, float y_ccw_true,
                           float x_cw_true, float y_cw_true) {
  float x, y;
  int num_intersections;
  int result = 0;

  // Validate counter-clockwise configuration results
  x = 0.0f;
  y = 0.0f;
  num_intersections = CircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2, 1.0f, x, y);
//  printf("< x , y > = < %f , %f >\t|\tnum_intersections = %d\n", x, y, num_intersections);
  if ((num_intersections == 0 && num_intersections_true == 2) ||
      (num_intersections == 2 && num_intersections_true == 0)) {
    result |= (1 << 0);
  }
  if (!FloatIsClose(x, x_ccw_true)) result |= (1 << 1);
  if (!FloatIsClose(y, y_ccw_true)) result |= (1 << 2);

  // Validate clockwise configuration results
  x = 0.0f;
  y = 0.0f;
  num_intersections = CircleIntersection(c1_x, c1_y, r1, c2_x, c2_y, r2, -1.0f, x, y);
//  printf("< x , y > = < %f , %f >\t|\tnum_intersections = %d\n", x, y, num_intersections);
  if ((num_intersections == 0 && num_intersections_true == 2) ||
      (num_intersections == 2 && num_intersections_true == 0)) {
    result |= (1 << 3);
  }
  if (!FloatIsClose(x, x_cw_true)) result |= (1 << 4);
  if (!FloatIsClose(y, y_cw_true)) result |= (1 << 5);

  return result;
}


int TestInverseKinematics(
    float x, float y, float z,
    float joint_1_plane_offset, float linkage_2_length, float linkage_3_length,
    float j1_config, float j2_config,
    int reachable_true,
    float j1_angle_true, float j2_angle_true, float j3_angle_true) {
  float j1_angle, j2_angle, j3_angle;
  int reachable;
  int result = 0;

  reachable = InverseKinematics(x, y, z,
                                joint_1_plane_offset,
                                linkage_2_length,
                                linkage_3_length,
                                j1_config, j2_config,
                                j1_angle, j2_angle, j3_angle);
//  printf("< j1, j2, j3 > = < %f , %f , %f >\treachable = %d\n", j1_angle, j2_angle, j3_angle, reachable);
  if (reachable != reachable_true) result |= (1 << 0);
  if (!FloatIsClose(j1_angle, j1_angle_true)) result |= (1 << 1);
  if (!FloatIsClose(j2_angle, j2_angle_true)) result |= (1 << 2);
  if (!FloatIsClose(j3_angle, j3_angle_true)) result |= (1 << 3);

  return result;
}
