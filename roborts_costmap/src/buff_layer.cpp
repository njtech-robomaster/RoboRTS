#include "buff_layer.h"
#include "buff_layer_setting.pb.h"

namespace roborts_costmap {

void BuffLayer::OnInitialize() {
  ros::NodeHandle nh;
  is_current_ = true;
  ParaBuffLayer para_buff_layer;

  std::string buff_map = ros::package::getPath("roborts_costmap") + \
      "/config/buff_layer_config.prototxt";

  roborts_common::ReadProtoFromTextFile(buff_map.c_str(), &para_buff_layer);
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  first_map_only_ = para_buff_layer.first_map_only();
  subscribe_to_updates_ = para_buff_layer.subscribe_to_updates();
  use_maximum_ = para_buff_layer.use_maximum();
  buff_zone_smaller_ = para_buff_layer.buff_zone_smaller();

  map_received_ = false;
  bool is_debug_ = para_buff_layer.is_debug();
  map_topic_ = para_buff_layer.topic_name();
  map_sub_ =
      nh.subscribe(map_topic_.c_str(), 1, &BuffLayer::InComingMap, this);

  ros::Rate temp_rate(10);
  while (!map_received_) {
    ros::spinOnce();
    temp_rate.sleep();
  }

  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;

  BuffZone buff_zone{};
  buff_zone.max_x_ = -3.270;
  buff_zone.max_y_ = 0.790;
  buff_zone.min_x_ = -3.810;
  buff_zone.min_y_ = 0.310;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(1, buff_zone));

  buff_zone.max_x_ = -1.870;
  buff_zone.max_y_ = -0.350;
  buff_zone.min_x_ = -2.410;
  buff_zone.min_y_ = -0.830;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(2, buff_zone));

  buff_zone.max_x_ = 0.270;
  buff_zone.max_y_ = 2.035;
  buff_zone.min_x_ = -0.270;
  buff_zone.min_y_ = 1.555;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(3, buff_zone));

  buff_zone.max_x_ = 3.810;
  buff_zone.max_y_ = -0.310;
  buff_zone.min_x_ = 3.270;
  buff_zone.min_y_ = -0.790;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(6, buff_zone));

  buff_zone.max_x_ = 2.410;
  buff_zone.max_y_ = 0.830;
  buff_zone.min_x_ = 1.870;
  buff_zone.min_y_ = 0.350;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(5, buff_zone));

  buff_zone.max_x_ = 0.270;
  buff_zone.max_y_ = -1.555;
  buff_zone.min_x_ = -0.270;
  buff_zone.min_y_ = -2.035;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(4, buff_zone));

  buff_zone_sub_ = nh.subscribe("/game_zone_array_status", 1,
                                &BuffLayer::UpdateGameZoneArray, this);
}

void BuffLayer::Activate() { OnInitialize(); }

void BuffLayer::Deactivate() { map_sub_.shutdown(); }

void BuffLayer::Reset() {
  if (first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void BuffLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j,
                              int max_i, int max_j) {
  if (!map_received_) {
    return;
  }
  if (!layered_costmap_->IsRollingWindow()) {
    if (!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0),
                           temp_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for (auto i = min_i; i < max_i; ++i) {
      for (auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if (World2Map(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          } else {
            master_grid.SetCost(
                i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

void BuffLayer::UpdateBounds(double robot_x, double robot_y, double robot_yaw,
                               double *min_x, double *min_y, double *max_x,
                               double *max_y) {
  double wx, wy;
  if (!layered_costmap_->IsRollingWindow()) {
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  // just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_ + width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;
}

void BuffLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if (!layered_costmap_->IsRolling() &&
      (master_map->GetSizeXCell() != size_x ||
       master_map->GetSizeYCell() != size_y ||
       master_map->GetResolution() != resolution ||
       master_map->GetOriginX() != origin_x ||
       master_map->GetOriginY() != origin_y ||
       !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y,
                                true);
  } else if (size_x_ != size_x || size_y_ != size_y ||
             resolution_ != resolution || origin_x_ != origin_x ||
             origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      costmap_[temp_index] = FREE_SPACE;
      ++temp_index;
    }
  }

  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

void BuffLayer::UpdateGameZoneArray(
    const roborts_msgs::GameZoneArray &new_game_zone_array) {
  if (!map_received_) {
    return;
  }
  auto fill_buff_zone = [&](BuffZone buff_zone, unsigned char value) {
    unsigned int map_min_x;
    unsigned int map_min_y;
    unsigned int map_max_x;
    unsigned int map_max_y;
    World2Map(buff_zone.min_x_ + buff_zone_smaller_,
              buff_zone.min_y_ + buff_zone_smaller_, map_min_x, map_min_y);
    World2Map(buff_zone.max_x_ - buff_zone_smaller_,
              buff_zone.max_y_ - buff_zone_smaller_, map_max_x, map_max_y);
    for (unsigned int i = map_min_x; i < map_max_x; ++i) {
      for (unsigned int j = map_min_y; j < map_max_y; ++j) {
        if (i < 0 || i >= size_x_) {
          continue;
        }
        if (j < 0 || j >= size_y_) {
          continue;
        }
        costmap_[GetIndex(i, j)] = value;
        // ROS_WARN("%d", value);
      }
    }
  };

  // std::cout << "Robot color: " << robot_color_ << std::endl;

  std::vector<bool> vec_buff_active;
  vec_buff_active.emplace_back(new_game_zone_array.zone[0].active);
  vec_buff_active.emplace_back(new_game_zone_array.zone[1].active);
  vec_buff_active.emplace_back(new_game_zone_array.zone[2].active);
  vec_buff_active.emplace_back(new_game_zone_array.zone[3].active);
  vec_buff_active.emplace_back(new_game_zone_array.zone[4].active);
  vec_buff_active.emplace_back(new_game_zone_array.zone[5].active);

  std::vector<uint8_t> vec_buff_debuff_status;
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[0].type);
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[1].type);
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[2].type);
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[3].type);
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[4].type);
  vec_buff_debuff_status.emplace_back(
      new_game_zone_array.zone[5].type);

  std::string team_color;
  if (!ros::param::getCached("team_color", team_color)) {
    ROS_WARN("team_color is not set");
  }

  if (vec_last_buff_debuff_status_ != vec_buff_debuff_status ||
      vec_last_buff_active_ != vec_buff_active) {
    for (int i = 1; i <= 6; i++) {
      uint8_t status = vec_buff_debuff_status.at(i - 1);
      bool active = vec_buff_active.at(i - 1);
      BuffZone buff_zone = map_buff_zones_.at(i);
      if (active) {
        if (status == roborts_msgs::GameZone::RED_HP_RECOVERY || status == roborts_msgs::GameZone::RED_BULLET_SUPPLY) {
          if (team_color == "red") {
            fill_buff_zone(buff_zone, FREE_SPACE);
          } else if (team_color == "blue") {
            fill_buff_zone(buff_zone, BUFF_OBSTACLE);
          }
        } else if (status == roborts_msgs::GameZone::BLUE_HP_RECOVERY || status == roborts_msgs::GameZone::BLUE_BULLET_SUPPLY) {
          if (team_color == "red") {
            fill_buff_zone(buff_zone, BUFF_OBSTACLE);
          } else if (team_color == "blue") {
            fill_buff_zone(buff_zone, FREE_SPACE);
          }
        } else if (status == roborts_msgs::GameZone::DISABLE_SHOOTING || status == roborts_msgs::GameZone::DISABLE_MOVEMENT) {
          fill_buff_zone(buff_zone, BUFF_OBSTACLE);
        }
      } else {
        fill_buff_zone(buff_zone, FREE_SPACE);
      }
    }
    std::cout << "Update buff zone status successfully!" << std::endl;
  }

  vec_last_buff_debuff_status_ = vec_buff_debuff_status;
  vec_last_buff_active_ = vec_buff_active;
}

} // namespace roborts_costmap
