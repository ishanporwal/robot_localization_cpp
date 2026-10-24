
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "pf.hpp"

class ParticleFilterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    pf = std::make_shared<ParticleFilter>();
    pf->transform_helper_ = std::make_shared<TFHelper>(pf);
  }

  geometry_msgs::msg::Pose pose(float x, float y, float theta) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.orientation = quaternion_from_euler(0, 0, theta);
    return p;
  }

  void set_particles(const std::vector<Particle>& particles) {
    pf->particle_cloud = particles;
    pf->n_particles = particles.size();
  }

  bool moved_far_enough(std::vector<float> new_pose) {
    return pf->moved_far_enough_to_update(new_pose);
  }

  std::vector<Particle>& particles() { return pf->particle_cloud; }

  void initialize() { pf->initialize_particle_cloud(std::vector<float>{2, 3, 0}); }

  void set_odom(float x, float y, float angle) {
    pf->odom_pose = pose(x, y, angle);
  }

  void set_previous_odom(float x, float y, float angle) {
    pf->current_odom_xy_theta = {x, y, angle};
  }

  void update_odom() { pf->update_particles_with_odom(); }
  void update_laser() { pf->update_particles_with_laser(std::vector<float>(9, 2.0f),
                                                       std::vector<float>(9, 0.0f)); }
  void update_pose() { pf->update_robot_pose(); }
  void resample() { pf->resample_particles(); }

  void make_test_map() {
    auto field = std::make_shared<OccupancyField>();
    field->map.info.width = 20;
    field->map.info.height = 20;
    field->map.info.resolution = 1.0f;
    field->closest_occ = Matrix::Constant(20, 20, 1.0);
    field->closest_occ(3, 1) = 0.0;
    pf->occupancy_field = field;
  }

  bool has_estimate() { return pf->transform_helper_->translation.has_value(); }
  double estimated_x() { return pf->transform_helper_->translation->x; }
  double estimated_y() { return pf->transform_helper_->translation->y; }

  std::shared_ptr<ParticleFilter> pf;
};

TEST_F(ParticleFilterTest, Initialization) {
  initialize();
  ASSERT_EQ(particles().size(), 300u);
  float total = 0;
  for (const auto& p : particles()) {
    EXPECT_NEAR(p.w, 1.0 / 300.0, 1e-6);
    total += p.w;
  }
  EXPECT_NEAR(total, 1.0, 1e-4);
}

TEST_F(ParticleFilterTest, AngleWraparound) {
  set_previous_odom(0, 0, 3.13f);

  // Moving from +179 degrees to -179 degrees
  // should not trigger an update.
  EXPECT_FALSE(moved_far_enough({0, 0, -3.13f}));
}


TEST_F(ParticleFilterTest, Odometry) {
  constexpr float half_pi = 1.5707963f;
  set_particles(std::vector<Particle>(300, Particle(1.0f / 300.0f, half_pi, 2, 3)));
  set_previous_odom(0, 0, 0);
  set_odom(1, 0, 0);
  update_odom();

  double average_x = 0, average_y = 0;
  for (const auto& p : particles()) {
    average_x += p.x;
    average_y += p.y;
  }
  EXPECT_NEAR(average_x / particles().size(), 2.0, 0.08);
  EXPECT_NEAR(average_y / particles().size(), 4.0, 0.08);
}

TEST_F(ParticleFilterTest, LaserUpdate) {
  make_test_map();
  set_particles({Particle(0.5f, 0, 1.5f, 1.5f),
                 Particle(0.5f, 0, 1.5f, 4.5f)});
  update_laser();

  EXPECT_GT(particles()[0].w, particles()[1].w);
  EXPECT_NEAR(particles()[0].w + particles()[1].w, 1.0, 1e-5);
}

TEST_F(ParticleFilterTest, PoseEstimate) {
  set_particles({Particle(0.75f, 0, 2, 4),
                 Particle(0.25f, 0, 6, 8)});
  set_odom(0, 0, 0);
  update_pose();

  ASSERT_TRUE(has_estimate());
  EXPECT_NEAR(estimated_x(), 3.0, 1e-5);
  EXPECT_NEAR(estimated_y(), 5.0, 1e-5);
}

TEST_F(ParticleFilterTest, Resampling) {
  set_particles({Particle(1.0f, 0, 5, 5),
                 Particle(0.0f, 0, 9, 9)});
  resample();

  ASSERT_EQ(particles().size(), 2u);
  for (const auto& p : particles()) {
    EXPECT_FLOAT_EQ(p.x, 5.0f);
    EXPECT_FLOAT_EQ(p.y, 5.0f);
    EXPECT_FLOAT_EQ(p.w, 0.5f);
  }
}
