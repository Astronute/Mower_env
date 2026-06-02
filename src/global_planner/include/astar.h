#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <cfloat>
#include <memory>
#include <cmath>
#include "Eigen/Dense"

struct Node{
    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };
    enum NODE_STATUS {
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };

    Node(){}

    Node(unsigned int _x, unsigned int _y):mx(_x), my(_y), f(FLT_MAX), parent(nullptr){}

    // 
    double g,h,f;
    // 地图坐标系
    unsigned int index;
    int mx,my; 
    
    DIRECTION direction{};
    NODE_STATUS status{};
    int steering_grade;
    std::shared_ptr<Node> parent;

};

class AStar{
public:
    struct cmp {
        bool operator()(const std::shared_ptr<Node> a, const std::shared_ptr<Node> b) const {
            return a->f < b->f;
        }
    };

    AStar();

    AStar(std::vector<std::vector<int>> gmap, const double &xcost, const double &ycost);

    void initGridmap(const std::vector<std::vector<int>> &gmap);

    std::vector<std::shared_ptr<Node>> Expansion(const std::shared_ptr<Node> &node);

    bool makePlan(const Eigen::Vector2i &start, const Eigen::Vector2i &goal, std::vector<Eigen::Vector2i> &path);

    double ComputeG(const std::shared_ptr<Node> &cur, const std::shared_ptr<Node> &neighbor);

    double ComputeH(const std::shared_ptr<Node> &cur, const std::shared_ptr<Node> &end);

    bool CheckCollision(const unsigned int &mx, const unsigned int &my, const unsigned int &emx, const unsigned int &emy);

    bool CheckCollision(const unsigned int &mx, const unsigned int &my);

    bool CheckBoundary(const unsigned int &mx, const unsigned int &my);

    bool findNearestPassable(const Eigen::Vector2i& point, Eigen::Vector2i& vpoint, unsigned int max_radius);

    void Reset();

    int width_, height_;
    double x_move_cost_, y_move_cost_;
    double traj_length=0;
    std::string frame_id_;
    bool initialized_ = false;
    std::multiset<std::shared_ptr<Node>, cmp> openlist;
    std::list<std::shared_ptr<Node>> closelist;
    std::vector<Eigen::Vector2d> path;

    std::vector<std::vector<std::shared_ptr<Node>>> state_node_map_;
    std::vector<std::vector<int>> grid_map_;
};

#endif