#ifndef __FIELDS_COVER_PLANNER_H__
#define __FIELDS_COVER_PLANNER_H__

// #include "common_include.h"
#include <cfloat>
#include <memory>
#include <unordered_map>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
// #include "spline_planner.h"
#include "astar.h"
#include "Eigen/Dense"
#include "geometry_utils.h"
#include "polygon_offset.h"

#if(1)
#define FC_DEBUG
#endif


struct SwathLine{
    std::vector<GU::Point> navline; // 一条线段的点集，
    double theta;
    int line_priority;
    bool visited = false;
};

struct RingLine{
    std::vector<GU::Point> navline; // 一条线段的点集，
    int line_priority;
    bool visited = false;
};

struct RingNode{
    RingNode(){}

    RingNode(int id_, const std::vector<GU::Point> & points){
        id = id_;
        navline = points;
    }

    RingNode(const std::vector<GU::Point> && points){
        navline = std::move(points);
    }

    int id = 0;

    std::vector<GU::Point> navline;

    bool visited = false;

    std::vector<RingNode *> child;
};

struct PolygonNode{
    GU::Point p;
    
    bool is_intersection = false;

    PolygonNode * next = nullptr;

    PolygonNode * prev = nullptr;

    PolygonNode * friend_n = nullptr;

    double dist; // alongA
};

class GridMap{
public:
    GridMap(const double &resolution){
        resolution_ = resolution;
    }

    ~GridMap(){
    }

    void fastInitMap(
        const cv::Mat &map, 
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs, 
        const int &scale_factor_, 
        const double &boundary_offset, 
        const double &safety_margin
    );

    std::vector<std::vector<int>> getGridmap_();

private:
    std::vector<std::vector<int>> gridmap_;

    int scale_factor_;

    int width_, height_;

    double resolution_; // 原图分辨率
};

class FieldsCoverPlanner{
public:
    FieldsCoverPlanner();

    ~FieldsCoverPlanner();

    void reset();

    std::vector<GU::Point> preparePolygon(const std::vector<GU::Point> & polygon, double gap);

    bool prepareFields(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs,
        std::vector<GU::Point> &filtered_boundary,
        std::vector<std::vector<GU::Point>> &filtered_static_obs
    );

    bool createMaps(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs
    );

    void drawRings(cv::Mat &map, const std::vector<GU::Point> &ring);

    void drawRings(const std::string &name, const std::vector<GU::Point> &ring);

    std::vector<std::vector<GU::Point>> processObstacles(
        const std::vector<GU::Point> &boundary, 
        const std::vector<GU::Point> &obs
    );

    std::vector<SwathLine> createSwaths(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs
    );

    std::vector<std::vector<GU::Point>> polygonOffset(const std::vector<GU::Point> &polygon, double dis);

    bool planCellpath(
        std::vector<SwathLine> swaths, 
        std::vector<GU::Point> &path, 
        const std::vector<GU::Point>& mboundary,
        const std::vector<std::vector<GU::Point>>& mstatic_obs, 
        const std::shared_ptr<AStar> &astar
    );

    bool planRingpath(
        std::vector<RingLine> &rings, 
        std::vector<GU::Point> &path, 
        const std::vector<GU::Point>& boundary,
        const std::vector<GU::Point>& static_obs, 
        const std::shared_ptr<AStar> &astar
    );

    bool planRingpath(
        RingNode * root, 
        std::vector<GU::Point> &path, 
        const std::vector<GU::Point>& boundary, 
        const std::vector<GU::Point>& static_obs, 
        const std::shared_ptr<AStar> &astar
    );

    // 弓字形规划
    bool planLPath(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs, 
        const double &theta, 
        std::vector<GU::Point> &opt_trajectory
    );

    // 回字形规划
    bool planOPath(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs, 
        const double &theta, 
        std::vector<GU::Point> &opt_trajectory
    );

    // 螺旋形规划
    bool planSPath(
        const std::vector<GU::Point> &boundary, 
        const std::vector<std::vector<GU::Point>> &static_obs, 
        const double &theta, 
        std::vector<GU::Point> &opt_trajectory
    );

    bool mapToWorld(const int &px, const int &py, double &wx, double &wy);

    bool worldToMap(const double &wx, const double &wy, int &px, int &py);

    void setSwathWidth(const double &width);

    void setStartPoint(const double x, const double y);

    void setIgnoreObstacles(bool ignore);

    GU::Point getStartpoint();

    bool isIgnoreObstacles();

    double getSwathWidth();

    void getAllParameters();

private:
    Eigen::Vector2d map_origin_;

    cv::Mat map_;

    std::vector<std::vector<unsigned char>> map_data_;

    double start_x_, start_y_;

    double map_width_, map_height_;

    int map_width_px_, map_height_py_;

    double resolution_ = 0.05; // 地图分辨率 (m/pixl)

    bool map_initialized = false;

    bool ignore_obstacle = false;

    double safety_margin_ = 0.5; // 障碍物膨胀因子 (m)

    std::vector<RingNode *> all_ring_node_;

    // 弓字形参数

    double swath_width_ = 2.0; // 作业宽度

    double search_step_x_ = 0.1; // x轴分辨率

    double search_step_y_ = 0.1; // y轴分辨率 (m)

    double boundary_offset_ = 0.1; // 边界内缩距离 (m) 必须大于resolution_

    double dp_epsilon_ = 5; // astar抽稀阈值 (pixl) default:5

    int resolution_astar_ = 2; // 降采样倍数 default:2

    // 回字形参数

    std::unordered_map<int, RingNode *> node_map_;

};



#endif