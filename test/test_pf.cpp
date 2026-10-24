
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "pf.hpp"

// Set up the test class so we can access our particle filter functions
class ParticleFilterTest : public ::testing::Test {
protected:
  // Start ROS before running tests and shut it down after
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Create a new particle filter and TF helper for each test
  void SetUp() override {
    pf = std::make_shared<ParticleFilter>();
    pf->transform_helper_ = std::make_shared<TFHelper>(pf);
  }

  // Helper to create a pose from x, y, and theta
  geometry_msgs::msg::Pose pose(float x, float y, float theta) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.orientation = quaternion_from_euler(0, 0, theta);
    return p;
  }

  // Set the particle cloud to specific particles for testing
  void set_particles(const std::vector<Particle>& particles) {
    pf->particle_cloud = particles;
    pf->n_particles = particles.size();
  }

  // Check if the robot has moved far enough to update the filter
  bool moved_far_enough(std::vector<float> new_pose) {
    return pf->moved_far_enough_to_update(new_pose);
  }

  // Helper to access the particles
  std::vector<Particle>& particles() { return pf->particle_cloud; }

  // Initialize particles around a known starting pose
  void initialize() { pf->initialize_particle_cloud(std::vector<float>{2, 3, 0}); }

  // Set the current and previous odometry poses
  void set_odom(float x, float y, float angle) {
    pf->odom_pose = pose(x, y, angle);
  }

  void set_previous_odom(float x, float y, float angle) {
    pf->current_odom_xy_theta = {x, y, angle};
  }

  // Helpers to call the functions we want to test
  void update_odom() { pf->update_particles_with_odom(); }
  void update_laser() { pf->update_particles_with_laser(std::vector<float>(9, 2.0f),
                                                       std::vector<float>(9, 0.0f)); }
  void update_pose() { pf->update_robot_pose(); }
  void resample() { pf->resample_particles(); }

  // Make a small fake map so we can test the laser update without a map server
  void make_test_map() {
    auto field = std::make_shared<OccupancyField>();
    field->map.info.width = 20;
    field->map.info.height = 20;
    field->map.info.resolution = 1.0f;
    field->closest_occ = Matrix::Constant(20, 20, 1.0);
    field->closest_occ(3, 1) = 0.0;
    pf->occupancy_field = field;
  }

  // Helpers to read the estimated robot position
  bool has_estimate() { return pf->transform_helper_->translation.has_value(); }
  double estimated_x() { return pf->transform_helper_->translation->x; }
  double estimated_y() { return pf->transform_helper_->translation->y; }

  std::shared_ptr<ParticleFilter> pf;
};

// Test 1: Check that we initialize the right number of particles with equal weights
TEST_F(ParticleFilterTest, Initialization) {
  initialize();

  // We should have 300 particles
  ASSERT_EQ(particles().size(), 300u);

  // Each particle should have a weight of 1/300, and all weights should sum to 1
  float total = 0;
  for (const auto& p : particles()) {
    EXPECT_NEAR(p.w, 1.0 / 300.0, 1e-6);
    total += p.w;
  }
  EXPECT_NEAR(total, 1.0, 1e-4);
}

// Test 2: Make sure crossing the +/- pi boundary doesn't trigger an update
TEST_F(ParticleFilterTest, AngleWraparound) {
  set_previous_odom(0, 0, 3.13f);

  // Moving from +179 degrees to -179 degrees
  // should not trigger an update.
  EXPECT_FALSE(moved_far_enough({0, 0, -3.13f}));
}


// Test 3: Make sure the odometry update moves particles in the right direction
TEST_F(ParticleFilterTest, Odometry) {
  constexpr float half_pi = 1.5707963f;

  // Start all particles at (2,3), facing 90 degrees
  set_particles(std::vector<Particle>(300, Particle(1.0f / 300.0f, half_pi, 2, 3)));
  set_previous_odom(0, 0, 0);

  // Move the robot forward 1 meter according to odometry
  set_odom(1, 0, 0);
  update_odom();

  // Average the particle positions since the motion model adds random noise
  double average_x = 0, average_y = 0;
  for (const auto& p : particles()) {
    average_x += p.x;
    average_y += p.y;
  }

  // Since the particles face 90 degrees, they should move in the y direction
  EXPECT_NEAR(average_x / particles().size(), 2.0, 0.08);
  EXPECT_NEAR(average_y / particles().size(), 4.0, 0.08);
}

// Test 4: Check that the laser update gives more weight to particles near obstacles
TEST_F(ParticleFilterTest, LaserUpdate) {
  make_test_map();

  // Start two particles at different locations with equal weights
  set_particles({Particle(0.5f, 0, 1.5f, 1.5f),
                 Particle(0.5f, 0, 1.5f, 4.5f)});
  update_laser();

  // The first particle should have a higher weight based on the fake map
  EXPECT_GT(particles()[0].w, particles()[1].w);

  // The weights should still sum to 1 after the update
  EXPECT_NEAR(particles()[0].w + particles()[1].w, 1.0, 1e-5);
}

// Test 5: Check that our estimated robot position is the weighted average of the particles
TEST_F(ParticleFilterTest, PoseEstimate) {
  // Two particles with different weights and locations
  set_particles({Particle(0.75f, 0, 2, 4),
                 Particle(0.25f, 0, 6, 8)});
  set_odom(0, 0, 0);
  update_pose();

  // Make sure the estimated pose actually exists
  ASSERT_TRUE(has_estimate());

  // Weighted average should give x = 3 and y = 5
  EXPECT_NEAR(estimated_x(), 3.0, 1e-5);
  EXPECT_NEAR(estimated_y(), 5.0, 1e-5);
}

// Test 6: Check that resampling picks particles with higher weights
TEST_F(ParticleFilterTest, Resampling) {
  // First particle has all the weight, second has zero
  set_particles({Particle(1.0f, 0, 5, 5),
                 Particle(0.0f, 0, 9, 9)});
  resample();

  // Resampling should keep the same number of particles
  ASSERT_EQ(particles().size(), 2u);

  // Both new particles should come from the first particle
  // and should have equal weights after resampling
  for (const auto& p : particles()) {
    EXPECT_FLOAT_EQ(p.x, 5.0f);
    EXPECT_FLOAT_EQ(p.y, 5.0f);
    EXPECT_FLOAT_EQ(p.w, 0.5f);
  }
}
