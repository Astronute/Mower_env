#include "fields_cover_planner.h"
#include <stack>
#include <algorithm>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Straight_skeleton_2/IO/print.h>
#include <cassert>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
typedef K::FT                        FT ;
typedef K::Point_2                   Point ;
typedef CGAL::Polygon_2<K>           Polygon_2 ;
typedef CGAL::Straight_skeleton_2<K> Ss ;
typedef std::shared_ptr<Polygon_2> PolygonPtr ;
typedef std::shared_ptr<Ss> SsPtr ;
typedef std::vector<PolygonPtr> PolygonPtrVector ;

const double PI = 3.1415926;


void GridMap::fastInitMap(
    const cv::Mat &map, const std::vector<GU::Point> &boundary, 
    const std::vector<std::vector<GU::Point>> &static_obs, 
    const int &scale_factor_, 
    const double &boundary_offset, 
    const double &safety_margin
){
    // map.cols << "x" << map.rows
    height_ = map.rows / scale_factor_;
    width_ = map.cols / scale_factor_;
    gridmap_.resize(height_);
    for(int i = 0; i < height_; ++i) {
        gridmap_[i] = std::vector<int>(width_, 0);
    }
    int len1 = 2 * boundary.size();
    double* fast_region = new double[len1];
    for (int i = 0; i < boundary.size(); ++i) {
        fast_region[2 * i] = boundary[i](0);
        fast_region[2 * i + 1] = boundary[i](1);
    }
    double* fast_obstacle = nullptr;
    int len2 = 0;
    if(!static_obs.empty()){
        len2 = 2 * static_obs[0].size();
        fast_obstacle = new double[len2];
        for (int i = 0; i < static_obs[0].size(); ++i) {
            fast_obstacle[2 * i] = static_obs[0][i](0);
            fast_obstacle[2 * i + 1] = static_obs[0][i](1);
        }
    }

    double boundary_offset_px = boundary_offset / resolution_;
    double obstacle_offset_px = safety_margin / resolution_;
    double px, py;
    for(int i=0; i<height_; ++i){
        for(int j=0; j<width_; ++j){
            px = j*scale_factor_;
            py = i*scale_factor_;
            double dis2boundary = GU::pointInPolygon(px, py, fast_region, len1, true);
            if(!static_obs.empty()){
                double dis2obs = GU::pointInPolygon(px, py, fast_obstacle, len2, true);
                if(dis2obs>=-obstacle_offset_px || dis2boundary <= boundary_offset_px){ //  && 
                    gridmap_[i][j] = 1;
                }
            }
            else{
                if(dis2boundary <= boundary_offset_px){ //  && 
                    gridmap_[i][j] = 1;
                }
            }

            // std::cout << gridmap_[i][j];
        }
        // std::cout << std::endl;
    }
    delete[] fast_region;
    delete[] fast_obstacle;
}

std::vector<std::vector<int>> GridMap::getGridmap_(){
    return gridmap_;
}

FieldsCoverPlanner::FieldsCoverPlanner(){
    map_origin_ << 0.0, 0.0;
}

FieldsCoverPlanner::~FieldsCoverPlanner(){
    for(auto* node : all_ring_node_) {
        delete node;
    }
    all_ring_node_.clear();
}

void FieldsCoverPlanner::reset(){
    for(auto* node : all_ring_node_) {
        delete node;
    }
    all_ring_node_.clear();
    node_map_.clear();
}

std::vector<GU::Point> FieldsCoverPlanner::preparePolygon(
    const std::vector<GU::Point> & polygon, 
    double gap
){
    std::vector<GU::Point> filter_vertex;
    
    int len = polygon.size();
    if (len < 3) {
        std::cout << " Invalid polygon: fewer than 3 vertices " << std::endl;
        return filter_vertex;
    }
    // |p1-p2|^2
    auto norm2 = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    };
    // |p1-p2|
    auto norm = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    };
    
    // update polygon list
    std::vector<PolygonNode*> all_polygon_node_;
    PolygonNode* root = nullptr;
    for(int i=0; i<polygon.size(); ++i){
        PolygonNode* node = new PolygonNode();
        all_polygon_node_.push_back(node);
        node->p = polygon[i];
        if(root == nullptr){
            node->next = node;
            node->prev = node;
            root = node;
        }
        else{
            PolygonNode* last_node = root->prev;
            last_node->next = node;
            root->prev = node;
            node->prev = last_node;
            node->next = root;
        }
    }
    // filter parallel/coincident
    PolygonNode* cur = root;
    PolygonNode* new_root = root;
    double dx1, dy1, dx2, dy2, cross;
    do {
        cur = cur->next;
        dx1 = cur->prev->p.x - cur->p.x; dy1 = cur->prev->p.y - cur->p.y;
        dx2 = cur->next->p.x - cur->p.x; dy2 = cur->next->p.y - cur->p.y;
        cross = dx1 * dy2 - dy1 * dx2;
        if (std::fabs(cross) < 1e-7 || norm(cur->p, cur->prev->p) < gap) {
            cur->prev->next = cur->next;
            cur->next->prev = cur->prev;
            if (cur == new_root) {
                new_root = cur->next;
            }
        }
    } while (cur != root);

    PolygonNode* p_vertex = new_root;
    do {
        filter_vertex.push_back(p_vertex->p);
        p_vertex = p_vertex->next;
    } while (p_vertex != new_root);

    for (auto node : all_polygon_node_) {
        delete node;
    }
    all_polygon_node_.clear();

    if (filter_vertex.size() < 3) {
        std::cout << " Invalid polygon: fewer than 3 vertices " << std::endl;
        filter_vertex.clear();
        return filter_vertex;
    }

    // filter selfintersection
    if (GU::calc_geometryIntersection(filter_vertex).size() > 0) {
        std::cout << " Invalid polygon: self intersection " << std::endl;
        filter_vertex.clear();
        return filter_vertex;
    }

    // ccw
    double area = GU::polygonArea(filter_vertex);
    if (area > 0) {
        std::reverse(filter_vertex.begin(), filter_vertex.end());
    }
    std::cout << "polygon area: " << area << std::endl;

    return filter_vertex;
}

bool FieldsCoverPlanner::prepareFields(
    const std::vector<GU::Point> &boundary, 
    const std::vector<std::vector<GU::Point>> &static_obs,
    std::vector<GU::Point> &filtered_boundary,
    std::vector<std::vector<GU::Point>> &filtered_static_obs
){
    // process boundary
    std::vector<GU::Point> new_boundary = preparePolygon(boundary, 0.1);
    if(new_boundary.size() < 3){
        std::cout << "Invalid boundary: fewer than 3 valid vertices" << std::endl;
        return false;
    }

    // process obstacle
    if(!isIgnoreObstacles()){
        for(auto obs: static_obs){
            std::vector<GU::Point> new_obs = preparePolygon(obs, 0.1);
            if(new_obs.size() < 3){
                std::cout << "Invalid obstacle: fewer than 3 valid vertices" << std::endl;
                return false;
            }
            else{
                if(GU::pointInPolygon(getStartpoint(), new_obs, false) != -1){
                    std::cout << "start point in obstacle" << std::endl;
                    return false;
                }
                filtered_static_obs.push_back(new_obs);
                // cut boundary
                std::vector<std::vector<GU::Point>> process_rings = GU::calc_AnotB(new_boundary, new_obs);
                if(process_rings.size() > 1){
                    std::cout << "Invalid obstacle: cross boundary" << std::endl;
                    return false;
                }
                else{
                    new_boundary = preparePolygon(process_rings[0], 0.1);
                    if(new_boundary.size() < 3){
                        return false;
                    }
                }
            }
        }

    }

    filtered_boundary = new_boundary;

    return true;
}

bool FieldsCoverPlanner::mapToWorld(const int &px, const int &py, double &wx, double &wy){
    if(!map_initialized){
        std::cout << "The map has not been initialized, please call createMaps()" << std::endl;
        return false;
    }
    if(px < 0 || py < 0 || px > map_width_px_ || py > map_height_py_){
        return false;
    }
    wx = map_origin_(0) + px * resolution_;
    wy = map_origin_(1) + py * resolution_;
    return true;
}

bool FieldsCoverPlanner::worldToMap(const double &wx, const double &wy, int &px, int &py){
    if(!map_initialized){
        std::cout << "The map has not been initialized, please call createMaps()" << std::endl;
        return false;
    }
    px = static_cast<int>(std::round((wx - map_origin_(0))/resolution_));
    py = static_cast<int>(std::round((wy - map_origin_(1))/resolution_));
    // if(px < 0 || py < 0 || px > map_width_px_ || py > map_height_py_){
    //     return false;
    // }
    return true;
}

void FieldsCoverPlanner::setSwathWidth(const double &width){
    double width_clamped = std::clamp(width, 0.5, 5.0);
    swath_width_ = width_clamped;
    std::cout << "swath width set to: " << swath_width_ << std::endl;
}

void FieldsCoverPlanner::setStartPoint(const double x, const double y){
    start_x_ = x;
    start_y_ = y;
    std::cout << "start point set to: " << start_x_ << ", " << start_y_ << std::endl;
}

void FieldsCoverPlanner::setIgnoreObstacles(bool ignore){
    ignore_obstacle = ignore;
}

GU::Point FieldsCoverPlanner::getStartpoint(){
    return GU::Point(start_x_, start_y_);
}

double FieldsCoverPlanner::getSwathWidth(){
    return swath_width_;
}

bool FieldsCoverPlanner::isIgnoreObstacles(){
    return ignore_obstacle;
}

void FieldsCoverPlanner::getAllParameters(){

}

bool FieldsCoverPlanner::createMaps(const std::vector<GU::Point> &boundary, const std::vector<std::vector<GU::Point>> &static_obs){
    std::vector<std::vector<cv::Point>> fill_boundaries, fill_obstacles;
    std::vector<cv::Point> fill_boundary, fill_obstacle;
    // create background
    double boundary_min_x, boundary_max_x, boundary_min_y, boundary_max_y;
    boundary_min_x = boundary_max_x = boundary[0](0);
    boundary_min_y = boundary_max_y = boundary[0](1);
    if(boundary.size() < 3){
        std::cout << "boundary points illegal" << std::endl;
        return false;
    }
    for(const GU::Point &p: boundary){
        boundary_min_x = boundary_min_x>p(0)?p(0):boundary_min_x;
        boundary_max_x = boundary_max_x<p(0)?p(0):boundary_max_x;
        boundary_min_y = boundary_min_y>p(1)?p(1):boundary_min_y;
        boundary_max_y = boundary_max_y<p(1)?p(1):boundary_max_y;
    }
    map_origin_ << boundary_min_x, boundary_min_y;
    map_width_ = std::ceil(boundary_max_x - boundary_min_x);
    map_height_ = std::ceil(boundary_max_y - boundary_min_y);
    map_width_px_ = std::ceil(map_width_ / resolution_);
    map_height_py_ = std::ceil(map_height_ / resolution_);
    map_ = cv::Mat::zeros(map_height_py_, map_width_px_, CV_8UC1);
    map_initialized = true;

    // fill boundary
    for(const GU::Point &p: boundary){
        int px, py;
        worldToMap(p(0), p(1), px, py);
        fill_boundary.push_back(cv::Point(px, py));
    }
    fill_boundaries.push_back(fill_boundary);

    // fill obstacles
    if(!ignore_obstacle){
        for(std::vector<GU::Point> obs: static_obs){
            for(GU::Point vec: obs){
                int px, py;
                worldToMap(vec(0), vec(1), px, py);
                fill_obstacle.push_back(cv::Point(px, py));
            }
            fill_obstacles.push_back(fill_obstacle);
        }
    }

    cv::Mat mask = cv::Mat::zeros(map_.size(), CV_8UC1);
    cv::fillPoly(mask, fill_boundaries, cv::Scalar(255));
    cv::fillPoly(map_, fill_boundaries, cv::Scalar(255));
    cv::polylines(mask, fill_boundaries, true, cv::Scalar(0), 1);
    mask.copyTo(map_);
    if(!ignore_obstacle){
        cv::fillPoly(map_, fill_obstacles, cv::Scalar(0));
    }

#ifdef FC_DEBUG
        cv::imwrite("map.jpg", map_);
#endif
    
    return true;
}

void FieldsCoverPlanner::drawRings(cv::Mat &map, const std::vector<GU::Point> &ring){
    int len = ring.size();
    if(ring.size() < 3){
        return ;
    }

    for(int i=0; i<ring.size(); ++i){
        int mx, my;
        worldToMap(ring[i](0), ring[i](1), mx, my);
        Eigen::Vector2i pos_0(mx, my);
        worldToMap(ring[(i+1)%len](0), ring[(i+1)%len](1), mx, my);
        Eigen::Vector2i pos_1(mx, my);
        cv::line(map, cv::Point(int(pos_0(0)), int(pos_0(1))), cv::Point(int(pos_1(0)), int(pos_1(1))), cv::Scalar(0), 1);
    }
}

void FieldsCoverPlanner::drawRings(const std::string &name, const std::vector<GU::Point> &ring){

    int len = ring.size();
    if(ring.size() < 3){
        return ;
    }

    cv::Mat map = map_.clone();
    for(int i=0; i<ring.size(); ++i){
        int mx, my;
        worldToMap(ring[i](0), ring[i](1), mx, my);
        Eigen::Vector2i pos_0(mx, my);
        worldToMap(ring[(i+1)%len](0), ring[(i+1)%len](1), mx, my);
        Eigen::Vector2i pos_1(mx, my);
        cv::line(map, cv::Point(int(pos_0(0)), int(pos_0(1))), cv::Point(int(pos_1(0)), int(pos_1(1))), cv::Scalar(0), 1);
    }

}

std::vector<std::vector<GU::Point>> FieldsCoverPlanner::processObstacles(const std::vector<GU::Point> &boundary, const std::vector<GU::Point> &obs){
    std::vector<std::vector<GU::Point>> valid_rings;

    std::vector<std::vector<GU::Point>> process_rings = GU::calc_AnotB(boundary, obs);
    // std::cout << "GU ring num: " << process_rings.size() << std::endl;
    for(auto ring: process_rings){
        GU::sort_polygon_vertices_ccw(ring);
        if(std::fabs(GU::polygonArea(ring)) > 1.0){
            valid_rings.push_back(ring);
        }
    }
    
    return valid_rings;
}

std::vector<SwathLine> FieldsCoverPlanner::createSwaths(const std::vector<GU::Point> &boundary, const std::vector<std::vector<GU::Point>> &static_obs){
    double boundary_offset_px = boundary_offset_ / resolution_;
    double obstacle_offset_px = safety_margin_ / resolution_;
    int cur_x = 0;
    int cur_y = 0;
    int swath_step = swath_width_ / resolution_;
    int x_step = search_step_x_ / resolution_;
    int y_step = search_step_y_ / resolution_;
    int x_step_count = map_width_px_ / x_step;
    int y_step_count = map_height_py_ / y_step;
    bool first_swath = false;
    int l_priority = 0;
    std::vector<SwathLine> swaths;
    std::vector<SwathLine> swaths_obs_base;
    std::vector<SwathLine> swaths_fix_x;
    std::unordered_map<int, std::vector<SwathLine>> map_swaths;
    bool reverse_flag = false;

    int len1 = 2 * boundary.size();
    double* fast_region = new double[len1];
    for (int i = 0; i < boundary.size(); ++i) {
        fast_region[2 * i] = boundary[i](0);
        fast_region[2 * i + 1] = boundary[i](1);
    }
    double* fast_obstacle = nullptr;
    int len2 = 0;
    if(!static_obs.empty() && !ignore_obstacle){
        len2 = 2 * static_obs[0].size();
        fast_obstacle = new double[len2];
        for (int i = 0; i < static_obs[0].size(); ++i) {
            fast_obstacle[2 * i] = static_obs[0][i](0);
            fast_obstacle[2 * i + 1] = static_obs[0][i](1);
        }
    }

    while(cur_x < map_width_px_){
        cur_y = 0;
        SwathLine sline;
        for(int j=0; j<y_step_count; ++j){
            double dis2boundary = GU::pointInPolygon(cur_x, cur_y, fast_region, len1, true);
            if(dis2boundary > boundary_offset_px){
                if(!first_swath){
                    first_swath = true;
                }
                sline.line_priority = l_priority;
                sline.theta = !reverse_flag?PI / 2:(-PI / 2);
                sline.navline.push_back(GU::Point(cur_x, cur_y));
            }
            else{
                if(sline.navline.size() >= 2){
                    if(reverse_flag){
                        reverse(sline.navline.begin(), sline.navline.end());
                    }
                    swaths.push_back(sline);
                }
                sline.navline.clear();
            }
            cur_y += y_step;
        }
        if(sline.navline.size() >= 2){
            if(reverse_flag){
                reverse(sline.navline.begin(), sline.navline.end());
            }
            swaths.push_back(sline);
        }
        if(!first_swath){
            cur_x += x_step;
        }else{
            cur_x += swath_step;
        }
        l_priority++;
        reverse_flag = !reverse_flag;
    }

    if(!ignore_obstacle){
        for(int i=0; i<swaths.size(); ++i){
            SwathLine sline;
            for(int j=0; j<swaths[i].navline.size(); ++j){
                double dis2obs = GU::pointInPolygon(swaths[i].navline[j](0), swaths[i].navline[j](1), fast_obstacle, len2, true);
                // std::cout << "swath priority: " << swaths[i].line_priority << " pos: " << swaths[i].navline[j](0) << ", " << swaths[i].navline[j](1) << std::endl;
                if(dis2obs < -1*obstacle_offset_px){
                    sline.line_priority = swaths[i].line_priority;
                    sline.theta = swaths[i].theta;
                    sline.navline.push_back(swaths[i].navline[j]);
                }
                else{
                    if(sline.navline.size() >=2){
                        // std::cout << "swath priority: " << sline.line_priority << "size : " << sline.navline.size() << std::endl;
                        swaths_obs_base.push_back(sline);
                    }
                    sline.navline.clear();
                }
            }
            if(sline.navline.size() >=2){
                swaths_obs_base.push_back(sline);
            }
        }
    }else{
        swaths_obs_base = swaths;
    }

    delete[] fast_region;
    delete[] fast_obstacle;

    return swaths_obs_base;
}

std::vector<std::vector<GU::Point>> FieldsCoverPlanner::polygonOffset(const std::vector<GU::Point> &polygon, double dis){
    std::vector<std::vector<GU::Point>> valid_rings;

    Polygon_2 poly;
    for(auto p: polygon){
        poly.push_back(Point(p(0), p(1)));
    }

    PolygonPtrVector offset_polygons;
    FT lOffset;
    if(dis>0){
        lOffset = dis;
        offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset, poly);
    }
    else{
        lOffset = -1.0*dis;
        offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(lOffset, poly);
    }
    
    for(auto pi = offset_polygons.begin() ; pi != offset_polygons.end() ; ++ pi ){
        if(dis<0 && pi == offset_polygons.begin()){
            continue;
        }
        const Polygon_2& polygon = **pi;
        std::vector<GU::Point> off_ring;
        for(auto vi = polygon.vertices_begin() ; vi != polygon.vertices_end() ; ++ vi ){
            off_ring.push_back(GU::Point(vi->x(), vi->y()));
        }
        valid_rings.push_back(off_ring);
        // if(std::fabs(GU::polygonArea(off_ring))>1.0){
        //     valid_rings.push_back(off_ring);
        // }
    }

    return valid_rings;
}

bool FieldsCoverPlanner::planCellpath(
    std::vector<SwathLine> swaths, 
    std::vector<GU::Point> &path, 
    const std::vector<GU::Point>& mboundary, 
    const std::vector<std::vector<GU::Point>>& mstatic_obs, 
    const std::shared_ptr<AStar> &astar
){

    std::unordered_set<int> swath_set;
    int num_swaths = swaths.size();
    if(num_swaths < 2){
        std::cout << "less swath" << std::endl;
        return false;
    }

    // search first point
    int first_swath_idx = 0;
    {
        double wx, wy;
        mapToWorld(swaths[0].navline[0].x, swaths[0].navline[0].y, wx, wy);
        double dis1 = (wx-start_x_)*(wx-start_x_) + (wy-start_y_)*(wy-start_y_);
        mapToWorld(swaths[num_swaths-1].navline[0].x, swaths[num_swaths-1].navline[0].y, wx, wy);
        double dis2 = (wx-start_x_)*(wx-start_x_) + (wy-start_y_)*(wy-start_y_);
        first_swath_idx = dis1<dis2?0:num_swaths-1;
    }
    
    GU::Point cur_pos = swaths[first_swath_idx].navline.back();
    path.push_back(swaths[first_swath_idx].navline[0]);
    path.push_back(cur_pos);
    SwathLine target_line = swaths[first_swath_idx];
    int target_index;

    for(int i=0; i<swaths.size(); ++i){
        if(i!=first_swath_idx)
            swath_set.insert(i);
    }

    // search point
    while(!swath_set.empty()){
        
        cur_pos = path.back();
        {
            double min_dis, min_dis_pri, dis, dis_pri, diff, dx1, dy1;
            int idx_prior, idx_dis;
            min_dis = FLT_MAX;
            min_dis_pri= FLT_MAX;
            for (const auto& s_idx : swath_set){
                dx1 = cur_pos.x - swaths[s_idx].navline[0].x;
                dy1 = cur_pos.y - swaths[s_idx].navline[0].y;
                dis = std::sqrt(dx1*dx1 + dy1*dy1);
                if(std::abs(target_line.line_priority - swaths[s_idx].line_priority) == 1){
                    diff = dis - min_dis_pri;
                    if(diff < -1e-10){
                        min_dis_pri = dis;
                        idx_prior = s_idx;
                    }
                }
                else{
                    diff = dis - min_dis;
                    if(diff < -1e-10){
                        min_dis = dis;
                        idx_dis = s_idx;

                    }
                }
            }
            if(min_dis_pri - min_dis < 2.5/resolution_){
                target_line = swaths[idx_prior];
                target_index = idx_prior;
            }
            else{
                target_line = swaths[idx_dis];
                target_index = idx_dis;
            }

        }

        swath_set.erase(target_index);
        // debug
        bool search_flag = false;
        if(!GU::calc_line_cross_polygon(GU::Line(cur_pos, target_line.navline[0]), mboundary)){
            if(!mstatic_obs.empty()&&!ignore_obstacle){
                search_flag = GU::calc_line_cross_polygon(GU::Line(cur_pos, target_line.navline[0]), mstatic_obs[0]);
            }
        }
        else{
            search_flag = true;
        }

        if(search_flag){
            Eigen::Vector2i pos_0(cur_pos(0)/resolution_astar_, cur_pos(1)/resolution_astar_);
            Eigen::Vector2i pos_1(target_line.navline[0](0)/resolution_astar_, target_line.navline[0](1)/resolution_astar_);
            std::vector<Eigen::Vector2i> search_path_i;
            std::vector<GU::Point> search_path;
            if(astar->makePlan(pos_0, pos_1, search_path_i)){
                for(int i=0; i<search_path_i.size(); ++i){
                    search_path.push_back(GU::Point(search_path_i[i](0)*resolution_astar_, search_path_i[i](1)*resolution_astar_));
                }
                search_path.push_back(target_line.navline[0]);
                std::vector<GU::Point> simplify_path = GU::simplifyCurve(search_path, dp_epsilon_);
                path.insert(path.end(), simplify_path.begin(), simplify_path.end());
            }
            else{
                return false;
            }
        }
        else{
            path.push_back(target_line.navline[0]);
        }
        path.push_back(target_line.navline.back());
    }

    return true;
}

bool FieldsCoverPlanner::planRingpath(
    std::vector<RingLine> &rings, 
    std::vector<GU::Point> &path, 
    const std::vector<GU::Point>& boundary, 
    const std::vector<GU::Point>& static_obs, 
    const std::shared_ptr<AStar> &astar
){
#ifdef FC_DEBUG
    cv::Mat map_debug = map_.clone();
#endif
    std::unordered_set<int> ring_set;
    int num_rings = rings.size();
    if(num_rings < 1){
        return false;
    }
    // search first point
    int first_ring_idx = 0;
    int first_point_idx = 0;
    double min_dis = FLT_MAX;
    for(int i=0; i< rings.size(); ++i){
        if(rings[i].line_priority == 0){
            for(int j=0; j<rings[i].navline.size(); ++j){
                double dis2 = (rings[i].navline[j].x-start_x_)*(rings[i].navline[j].x-start_x_) + (rings[i].navline[j].y-start_y_)*(rings[i].navline[j].y-start_y_);
                if(dis2 < min_dis){
                    first_ring_idx = i;
                    first_point_idx = j;
                    min_dis = dis2;
                }
            }
        }
    }
    
    auto addRing = [&](int r_idx, int p_idx){
        int iter_idx = p_idx;
        int rp_num = rings[r_idx].navline.size();
        do{
            path.push_back(rings[r_idx].navline[iter_idx]);
            iter_idx = (iter_idx+1)%rp_num;
        }while(iter_idx!=p_idx);
        path.push_back(rings[r_idx].navline[iter_idx]);
    };
    
    for(int i=1; i<num_rings; ++i){
        ring_set.insert(i);
    }

    // search point
    path.clear();
    addRing(first_ring_idx, first_point_idx);
    while(!ring_set.empty()){
        GU::Point cur_pos = path.back();
        int tar_ring, tar_idx;
        double min_dis = FLT_MAX;
        for (const auto& r_idx : ring_set) {
            for(int i=0; i<rings[r_idx].navline.size(); ++i){
                double dis = std::sqrt((cur_pos.x-rings[r_idx].navline[i].x)*(cur_pos.x-rings[r_idx].navline[i].x) + 
                                        (cur_pos.y-rings[r_idx].navline[i].y)*(cur_pos.y-rings[r_idx].navline[i].y));
                if(dis < min_dis){
                    min_dis = dis;
                    tar_ring = r_idx;
                    tar_idx = i;
                }
            }
        }
        ring_set.erase(tar_ring);

        // connect
        bool search_flag = false;
        GU::Line jump_line = GU::Line(cur_pos, rings[tar_ring].navline[tar_idx]);
        if(!GU::calc_line_cross_polygon(jump_line, boundary)){
            if(!ignore_obstacle){
                search_flag = GU::calc_line_cross_polygon(jump_line, static_obs);
            }
        }
        else{
            search_flag = true;
        }

        if(search_flag){
            int mx, my;
            worldToMap(cur_pos(0), cur_pos(1), mx, my);
            Eigen::Vector2i pos_0(mx, my);
            worldToMap(rings[tar_ring].navline[tar_idx](0), rings[tar_ring].navline[tar_idx](1), mx, my);
            Eigen::Vector2i pos_1(mx, my);
            std::vector<Eigen::Vector2i> search_path_i;
            std::vector<GU::Point> search_path;
#ifdef FC_DEBUG
            int thickness = 2; // 粗细
            int radius = 3;    //（半径）
            cv::circle(map_debug, cv::Point(int(pos_0(0)), int(pos_0(1))), radius, cv::Scalar(0), thickness);
            cv::line(map_debug, cv::Point(int(pos_0(0)), int(pos_0(1))), cv::Point(int(pos_1(0)), int(pos_1(1))), cv::Scalar(0), 1);
#endif
            if(astar->makePlan(
                Eigen::Vector2i(pos_0(0)/resolution_astar_, pos_0(1)/resolution_astar_), 
                Eigen::Vector2i(pos_1(0)/resolution_astar_, pos_1(1)/resolution_astar_), 
                search_path_i)
            ){
                // std::cout << "plan size: " << search_path.size() << std::endl;
                for(int i=0; i<search_path_i.size(); ++i){
                    double wx, wy;
                    mapToWorld(search_path_i[i](0)*resolution_astar_, search_path_i[i](1)*resolution_astar_, wx, wy);
                    search_path.push_back(GU::Point(wx, wy));
                }
                // search_path.push_back(cur_pos);
                std::vector<GU::Point> simplify_path = GU::simplifyCurve(search_path, dp_epsilon_*resolution_);
                // std::cout << "simplify size: " << simplify_path.size() << std::endl;
                if(simplify_path.size() > 2){
                    path.insert(path.end(), simplify_path.begin()+1, simplify_path.end()-1);
                }
                
                addRing(tar_ring, tar_idx);
            }else{
                return false;
            }
        }
        else{
            addRing(tar_ring, tar_idx);
        }

    }
#ifdef FC_DEBUG
    cv::imwrite("map_debug.jpg", map_debug);
#endif

    return true;
}

bool FieldsCoverPlanner::planRingpath(
    RingNode * root, 
    std::vector<GU::Point> &path, 
    const std::vector<GU::Point>& boundary, 
    const std::vector<GU::Point>& static_obs, 
    const std::shared_ptr<AStar> &astar
){
    if(!root){
        return false;
    }

#ifdef FC_DEBUG
    cv::Mat map_debug = map_.clone();
#endif

    auto addRing = [&](std::vector<GU::Point> ring, int p_idx){
        int iter_idx = p_idx;
        int rp_num = ring.size();
        do{
            path.push_back(ring[iter_idx]);
            iter_idx = (iter_idx+1)%rp_num;
        }while(iter_idx!=p_idx);
        path.push_back(ring[iter_idx]);
    };

    auto searchClosestIdx = [&](std::vector<GU::Point> ring, GU::Point p) -> std::pair<int, double> {
        int idx = 0;
        double min_dis = FLT_MAX;
        for(int i=0; i<ring.size(); ++i){
            double dis2 = (ring[i].x-p.x)*(ring[i].x-p.x) + (ring[i].y-p.y)*(ring[i].y-p.y);
            if(dis2 < min_dis){
                min_dis = dis2;
                idx = i;
            }
        }
        return std::pair<int, double>(idx, min_dis);
    };

    auto isConvex = [&](GU::Point p, GU::Point left, GU::Point right) -> bool {
        double dx1, dy1, dx2, dy2, cross;
        dx1 = p.x - left.x; dy1 = p.y - left.y;
        dx2 = right.x - p.x; dy2 = right.y - p.y;
        cross = dx1 * dy2 - dy1 * dx2;
        return cross > 0;
    };

    GU::Point cur_point = getStartpoint();
    int current_ring_idx = 0;

    while(!node_map_.empty()){
        int cur_start_idx, child_start_idx;
        RingNode* current_ring = nullptr;
        RingNode* child_ring = nullptr;
        bool jump_out = false;

        std::cout << "cur_ring_idx: " << current_ring_idx << std::endl;

        // 1、current ring
        current_ring = node_map_[current_ring_idx];
        cur_start_idx = searchClosestIdx(current_ring->navline, cur_point).first;
        bool is_convex = false;
        do{
            int len = current_ring->navline.size();
            is_convex = isConvex(
                current_ring->navline[cur_start_idx], 
                current_ring->navline[(cur_start_idx - 1 + len) % len], 
                current_ring->navline[(cur_start_idx + 1) % len]
            );
            
            if(!is_convex){
                path.push_back(current_ring->navline[cur_start_idx]);
                cur_start_idx = (cur_start_idx + 1) % len;
                std::cout << "not convex:" << current_ring->navline[cur_start_idx] << std::endl;
                std::cout << "not convex:" << current_ring->navline[(cur_start_idx - 1 + len) % len] << std::endl;
                std::cout << "not convex:" << current_ring->navline[(cur_start_idx + 1) % len] << std::endl;
            }
        } while(!is_convex);

        // 2.1、search child ring(current child node)
        if(current_ring->child.size() == 1){
            RingNode* child_node = current_ring->child[0];
            if(node_map_.count(child_node->id) > 0){ 
                child_ring = child_node;
            }
        }
        else{
            jump_out = true;
        }

        // 2.2、search (other node)
        if(child_ring == nullptr){
            double min_dis = FLT_MAX;
            int ring_idx = -1;
            int point_idx;
            for(auto it = node_map_.begin(); it != node_map_.end(); it++){
                if(it->first != current_ring_idx){
                    auto res = searchClosestIdx(it->second->navline, cur_point);
                    if(res.second < min_dis){
                        ring_idx = it->first;
                        point_idx = res.first;
                        min_dis = res.second;
                    }
                }
            }
            if(ring_idx != -1){
                child_ring = node_map_[ring_idx];
                GU::Line left = GU::Line(
                    child_ring->navline[point_idx], 
                    child_ring->navline[(point_idx+1)%child_ring->navline.size()]
                );
                GU::Line right = GU::Line(
                    child_ring->navline[point_idx], 
                    child_ring->navline[(point_idx - 1 + child_ring->navline.size())%child_ring->navline.size()]
                );
                double dis1 = GU::calc_pointSegmentDistance(
                    current_ring->navline[cur_start_idx],
                    left, true
                );
                double dis2 = GU::calc_pointSegmentDistance(
                    current_ring->navline[cur_start_idx],
                    right, true
                );
                std::cout << "dis1: " << dis1 << " dis2: " << dis2 << std::endl;
                if(dis1 < 1.1 * getSwathWidth() && dis2 < 1.1 * getSwathWidth() && std::fabs(dis1 - dis2) < 1e-3){
                    jump_out = false;
                }
            }
        }
        node_map_.erase(current_ring_idx);
        
        // 3、process target ring
        int len1 = current_ring->navline.size();
        if(child_ring != nullptr){
            int len2 = child_ring->navline.size();
            child_start_idx = searchClosestIdx(child_ring->navline, current_ring->navline[cur_start_idx]).first;
            if(!jump_out){
                GU::Line lineA = GU::Line(child_ring->navline[(child_start_idx + 1) % len2], child_ring->navline[child_start_idx]);
                bool terminated = false;
                for(int i=0; i<len1; ++i){
                    GU::Line lineB = GU::Line(current_ring->navline[(cur_start_idx + i) % len1], current_ring->navline[(cur_start_idx + i + 1) % len1]);
                    
                    if(i == len1 - 1){
                        auto inc = GU::calc_linesIntersect(lineA, lineB);
                        path.push_back(current_ring->navline[(cur_start_idx + i) % len1]);
                        if(inc.alongA > 1.0 && inc.alongB > 0 && inc.alongB <= 1.0){
                            path.push_back(inc.p);
                            current_ring_idx = child_ring->id;
                            cur_point = child_ring->navline[child_start_idx];

                            terminated = true;
                        }
                    }
                    else{
                        path.push_back(current_ring->navline[(cur_start_idx + i) % len1]);
                    }
                }
                
                if(!terminated){
                    path.push_back(current_ring->navline[cur_start_idx]);
                    current_ring_idx = child_ring->id;
                    cur_point = child_ring->navline[child_start_idx];
                }
            }
            else{
                // close current ring
                for(int i=0; i<current_ring->navline.size(); ++i){
                    path.push_back(current_ring->navline[(cur_start_idx + i) % len1]);
                }
                path.push_back(current_ring->navline[cur_start_idx]);

                // connect to "child" ring
                bool search_flag = false;
                if(!GU::calc_line_cross_polygon(GU::Line(path.back(), child_ring->navline[child_start_idx]), boundary)){
                    if(!ignore_obstacle){
                        search_flag = GU::calc_line_cross_polygon(GU::Line(path.back(), child_ring->navline[child_start_idx]), static_obs);
                    }
                }
                else{
                    search_flag = true;
                }

                if(search_flag){
                    int mx, my;
                    worldToMap(path.back()(0), path.back()(1), mx, my);
                    Eigen::Vector2i pos_0(mx, my);
                    worldToMap(child_ring->navline[child_start_idx](0), child_ring->navline[child_start_idx](1), mx, my);
                    Eigen::Vector2i pos_1(mx, my);
                    std::vector<Eigen::Vector2i> search_path_i;
                    std::vector<GU::Point> search_path;
#ifdef FC_DEBUG
                    int thickness = 2; // 粗细
                    int radius = 3;    //（半径）
                    cv::circle(map_debug, cv::Point(int(pos_0(0)), int(pos_0(1))), radius, cv::Scalar(0), thickness);
                    cv::line(map_debug, cv::Point(int(pos_0(0)), int(pos_0(1))), cv::Point(int(pos_1(0)), int(pos_1(1))), cv::Scalar(0), 1);
#endif
                    if(astar->makePlan(Eigen::Vector2i(pos_0(0)/resolution_astar_, pos_0(1)/resolution_astar_), Eigen::Vector2i(pos_1(0)/resolution_astar_, pos_1(1)/resolution_astar_), search_path_i)){
                        // std::cout << "plan size: " << search_path.size() << std::endl;
                        for(int i=0; i<search_path_i.size(); ++i){
                            double wx, wy;
                            mapToWorld(search_path_i[i](0)*resolution_astar_, search_path_i[i](1)*resolution_astar_, wx, wy);
                            search_path.push_back(GU::Point(wx, wy));
                        }
                        // search_path.push_back(cur_pos);
                        std::vector<GU::Point> simplify_path = GU::simplifyCurve(search_path, dp_epsilon_*resolution_);
                        std::cout << "simplify size: " << simplify_path.size() << std::endl;
                        path.insert(path.end(), simplify_path.begin(), simplify_path.end()-1);
                    }else{
                        return false;
                    }
                }

                // update parameter
                current_ring_idx = child_ring->id;
                cur_point = child_ring->navline[child_start_idx];
            }
        }
        else{
            // close last ring
            for(int i=0; i<current_ring->navline.size(); ++i){
                path.push_back(current_ring->navline[(cur_start_idx + i) % len1]);
            }
            path.push_back(current_ring->navline[cur_start_idx]);
            break;
        }
    }
#ifdef FC_DEBUG
    cv::imwrite("map_debug.jpg", map_debug);
#endif

    return true;
}

bool FieldsCoverPlanner::planLPath(
    const std::vector<GU::Point> &boundary, 
    const std::vector<std::vector<GU::Point>> &static_obs, 
    const double &theta, std::vector<GU::Point> &opt_trajectory
){
    clock_t time_begin_cpu = clock();
    
    // filter
    std::vector<GU::Point> F_boundary;
    std::vector<std::vector<GU::Point>> F_obstacles;
    if(static_obs.empty()){
        setIgnoreObstacles(true);
    }
    if(!prepareFields(boundary, static_obs, F_boundary, F_obstacles)){
        return false;
    }

    // 旋转
    std::vector<GU::Point> rotatedBoundary;
    std::vector<std::vector<GU::Point>> rotatedObstacles;
    std::vector<GU::Point> map_rBoundary;
    std::vector<std::vector<GU::Point>> map_rObstacles;
    Eigen::Matrix<double, 3, 3> R;
    R << std::cos(theta), -1.0*std::sin(theta), 0,
         std::sin(theta), std::cos(theta), 0,
         0, 0, 1;
    Eigen::Vector3d point3h, rpoint3h;
    
    point3h << start_x_, start_y_, 1;
    rpoint3h = R * point3h;
    start_x_ = rpoint3h(0);
    start_y_ = rpoint3h(1);
    for(const GU::Point &p: F_boundary){
        point3h << p(0), p(1), 1;
        rpoint3h = R*point3h;
        rotatedBoundary.push_back(GU::Point(rpoint3h(0), rpoint3h(1)));
    }
    if(!ignore_obstacle){
        for(const std::vector<GU::Point> &obs: F_obstacles){
            std::vector<GU::Point> rotatedObstacle;
            for(GU::Point vec: obs){
                point3h << vec(0), vec(1), 1;
                rpoint3h = R*point3h;
                rotatedObstacle.push_back(GU::Point(rpoint3h(0), rpoint3h(1)));
            }
            rotatedObstacles.push_back(rotatedObstacle);
        }
    }

    // 地图创建
    if(!createMaps(rotatedBoundary, rotatedObstacles)){
        std::cout << " create map failed" << std::endl;
        return false;
    }

    for(const GU::Point &p: F_boundary){
        point3h << p(0), p(1), 1;
        rpoint3h = R*point3h;
        int mx, my;
        worldToMap(rpoint3h(0), rpoint3h(1), mx, my);
        map_rBoundary.push_back(GU::Point(mx, my));
    }
    if(!ignore_obstacle){
        for(const std::vector<GU::Point> &obs: F_obstacles){
            std::vector<GU::Point> m_obs;
            for(GU::Point vec: obs){
                point3h << vec(0), vec(1), 1;
                rpoint3h = R*point3h;
                int mx, my;
                worldToMap(rpoint3h(0), rpoint3h(1), mx, my);
                m_obs.push_back(GU::Point(mx, my));
            }
            map_rObstacles.push_back(m_obs);
        }
    }

    // init path search algorithm
    GridMap gridmap = GridMap(resolution_); // down sample map
    gridmap.fastInitMap(map_, map_rBoundary, map_rObstacles, resolution_astar_, boundary_offset_, safety_margin_); 
    std::shared_ptr<AStar> astar_search = std::make_shared<AStar>(gridmap.getGridmap_(), 1.0, 1.0); // 移动代价 1, 1

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// plan ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // create swaths
    std::vector<SwathLine> swathlines = createSwaths(map_rBoundary, map_rObstacles);
    if(swathlines.size() < 1){
        std::cout << " less swathlines " << std::endl;
        return false;
    }
    
    // field cover path plan
    std::vector<GU::Point> opt_trajectory_r;
    if(!planCellpath(swathlines, opt_trajectory_r, map_rBoundary, map_rObstacles, astar_search)){
        return false;
    }
    

    // recorver
    opt_trajectory.clear();
    Eigen::Matrix<double, 3, 3> R_inv;
    R_inv << std::cos(-1.0*theta), -1.0*std::sin(-1.0*theta), 0,
         std::sin(-1.0*theta), std::cos(-1.0*theta), 0,
         0, 0, 1;
    for(int i=0; i<opt_trajectory_r.size(); ++i){
        double wx, wy;
        mapToWorld(opt_trajectory_r[i](0), opt_trajectory_r[i](1), wx, wy);
        point3h << wx, wy, 1;
        rpoint3h = R_inv*point3h;
        opt_trajectory.push_back(GU::Point(rpoint3h(0), rpoint3h(1)));
    }

    clock_t time_end_cpu111 = clock();
    double planning_duration = (double)(time_end_cpu111 - time_begin_cpu) / CLOCKS_PER_SEC;
    std::cout<<"planning duration: "<<planning_duration<<std::endl;

#ifdef FC_DEBUG
        cv::Mat map_with_path = map_.clone();
        map_with_path.setTo(cv::Scalar(255));
        int mx, my;
        for(int i=0; i<opt_trajectory_r.size()-1; ++i){
            cv::Point start_point(opt_trajectory_r[i](0), opt_trajectory_r[i](1));
            cv::Point end_point(opt_trajectory_r[i+1](0), opt_trajectory_r[i+1](1));
            cv::line(map_with_path, start_point, end_point, cv::Scalar(0), 1.5);
        }
        cv::imwrite("map_with_path.jpg", map_with_path);
#endif

    return true;
}

bool FieldsCoverPlanner::planOPath(
    const std::vector<GU::Point> &boundary, 
    const std::vector<std::vector<GU::Point>> &static_obs, 
    const double &theta, std::vector<GU::Point> &opt_trajectory
){
    clock_t time_begin_cpu = clock();

    // filter
    std::vector<GU::Point> F_boundary;
    std::vector<std::vector<GU::Point>> F_obstacles;
    if(static_obs.empty()){
        setIgnoreObstacles(true);
    }
    if(!prepareFields(boundary, static_obs, F_boundary, F_obstacles)){
        return false;
    }


    // 地图创建
    std::vector<GU::Point> map_Boundary;
    std::vector<std::vector<GU::Point>> map_Obstacles;
    std::vector<GU::Point> safe_obstacle;
    if(!createMaps(F_boundary, F_obstacles)){
        std::cout << " create map failed" << std::endl;
        return false;
    }
    for(const GU::Point &p: F_boundary){
        int mx, my;
        worldToMap(p(0), p(1), mx, my);
        map_Boundary.push_back(GU::Point(mx, my));
    }
    if(!ignore_obstacle){
        for(const std::vector<GU::Point> &obs: F_obstacles){
            std::vector<GU::Point> m_obs;
            safe_obstacle = polygonOffset(obs, -1.0*safety_margin_)[0]; // 障碍物膨胀
            for(GU::Point p: safe_obstacle){
                int mx, my;
                worldToMap(p(0), p(1), mx, my);
                m_obs.push_back(GU::Point(mx, my));
            }
            map_Obstacles.push_back(m_obs);
        }
    }

    // init path search algorithm
    GridMap gridmap = GridMap(resolution_); // down sample map
    gridmap.fastInitMap(map_, map_Boundary, map_Obstacles, resolution_astar_, boundary_offset_, safety_margin_); 
    std::shared_ptr<AStar> astar_search = std::make_shared<AStar>(gridmap.getGridmap_(), 1.0, 1.0); // 移动代价 1, 1
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// plan ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    int plan_count = 0;
    std::vector<RingLine> structured_rings;
    std::vector<std::vector<GU::Point>> target_rings{F_boundary};
    std::vector<std::vector<GU::Point>> new_rings;
    cv::Mat ring_map_debug = map_.clone();
    while(target_rings.size()){
        new_rings.clear();
        for(auto o_ring: target_rings){
            std::vector<std::vector<GU::Point>> valid_rings = polygonOffset(o_ring, swath_width_);
            if(valid_rings.size() < 1){
                valid_rings = polygonOffset(o_ring, 0.5 * swath_width_);
            }
            // std::vector<std::vector<GU::Point>> valid_rings = po.inflatePolygon(o_ring, swath_width_);
            for(auto ring: valid_rings){
                // process obstacle
                if(!ignore_obstacle){
                    std::vector<std::vector<GU::Point>> valid_rings_nobs = processObstacles(ring, safe_obstacle);
                    for(auto r:valid_rings_nobs){
                        RingLine rl;
                        rl.navline = r;
                        rl.line_priority = plan_count;
                        rl.visited = false;
                        drawRings(ring_map_debug, r);
                        structured_rings.push_back(rl);
                        new_rings.push_back(r);
                    }
                }
                else{
                    RingLine rl;
                    rl.navline = ring;
                    rl.line_priority = plan_count;
                    rl.visited = false;
                    structured_rings.push_back(rl);
                    new_rings.push_back(ring);
                }
            }
        }
        plan_count++;
        if(plan_count > 1000){
            return false;
        }
        // std::cout << " iter " << plan_count << "valid ring num: " << new_rings.size() << std::endl;
        target_rings = new_rings;
    }
    cv::imwrite("map_with_ring.jpg", ring_map_debug);

    // search path
    std::vector<GU::Point> inner_trajectory;
    if(structured_rings.size() < 1){
        std::cout << " planning zero rings " <<  std::endl;
        return false;
    }
    if(!planRingpath(structured_rings, inner_trajectory, F_boundary, safe_obstacle, astar_search)){
        return false;
    }
    
    {
        int gate_idx = 0;
        double min_dis = FLT_MAX;
        for(int i=0; i< F_boundary.size(); ++i){
            double dis2 = (F_boundary[i].x-start_x_)*(F_boundary[i].x-start_x_) + (F_boundary[i].y-start_y_)*(F_boundary[i].y-start_y_);
            if(dis2 < min_dis){
                gate_idx = i;
                min_dis = dis2;
            }
        }
        for(int i=0; i< F_boundary.size(); ++i){
            opt_trajectory.push_back(F_boundary[(gate_idx + i)%F_boundary.size()]);
        }
        opt_trajectory.push_back(F_boundary[gate_idx]);
        opt_trajectory.insert(opt_trajectory.end(), inner_trajectory.begin(), inner_trajectory.end());
    }

    clock_t time_end_cpu111 = clock();
    double planning_duration = (double)(time_end_cpu111 - time_begin_cpu) / CLOCKS_PER_SEC;
    std::cout<<"planning duration: "<<planning_duration<<std::endl;

#ifdef FC_DEBUG
    cv::Mat map_with_path = map_.clone();
    map_with_path.setTo(cv::Scalar(255));
    int mx, my;
    for(int i=0; i<inner_trajectory.size()-1; ++i){
        worldToMap(inner_trajectory[i](0), inner_trajectory[i](1), mx, my);
        cv::Point start_point(mx, my);
        worldToMap(inner_trajectory[i+1](0), inner_trajectory[i+1](1), mx, my);
        cv::Point end_point(mx, my);
        cv::line(map_with_path, start_point, end_point, cv::Scalar(0), 1.5);
    }
    cv::imwrite("map_with_path.jpg", map_with_path);
#endif

    return true;
}

bool FieldsCoverPlanner::planSPath(
    const std::vector<GU::Point> &boundary, 
    const std::vector<std::vector<GU::Point>> &static_obs, 
    const double &theta, std::vector<GU::Point> &opt_trajectory
){
    clock_t time_begin_cpu = clock();

    reset();

    // filter
    std::vector<GU::Point> F_boundary;
    std::vector<std::vector<GU::Point>> F_obstacles;
    if(static_obs.empty()){
        setIgnoreObstacles(true);
    }
    if(!prepareFields(boundary, static_obs, F_boundary, F_obstacles)){
        return false;
    }

    // 地图创建
    std::vector<GU::Point> map_Boundary;
    std::vector<std::vector<GU::Point>> map_Obstacles;
    std::vector<GU::Point> safe_obstacle;
    if(!createMaps(F_boundary, F_obstacles)){
        std::cout << " create map failed" << std::endl;
        return false;
    }
    for(const GU::Point &p: F_boundary){
        int mx, my;
        worldToMap(p(0), p(1), mx, my);
        map_Boundary.push_back(GU::Point(mx, my));
    }
    if(!ignore_obstacle){
        for(const std::vector<GU::Point> &obs: F_obstacles){
            std::vector<GU::Point> m_obs;
            safe_obstacle = polygonOffset(obs, -1.0*safety_margin_)[0]; // 障碍物膨胀
            for(GU::Point p: safe_obstacle){
                int mx, my;
                worldToMap(p(0), p(1), mx, my);
                m_obs.push_back(GU::Point(mx, my));
            }
            map_Obstacles.push_back(m_obs);
        }
    }

    // init path search algorithm
    GridMap gridmap = GridMap(resolution_); // down sample map
    gridmap.fastInitMap(map_, map_Boundary, map_Obstacles, resolution_astar_, boundary_offset_, safety_margin_); 
    std::shared_ptr<AStar> astar_search = std::make_shared<AStar>(gridmap.getGridmap_(), 1.0, 1.0); // 移动代价 1, 1
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// plan ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    int plan_count = 0;

    auto searchClosestIdx = [&](std::vector<GU::Point> ring, GU::Point p) -> int {
        int idx = 0;
        double min_dis = FLT_MAX;
        for(int i=0; i<ring.size(); ++i){
            double dis2 = (ring[i].x-p.x)*(ring[i].x-p.x) + (ring[i].y-p.y)*(ring[i].y-p.y);
            if(dis2 < min_dis){
                min_dis = dis2;
                idx = i;
            }
        }
        return idx;
    };
    int node_id = 0;
    RingNode* boundary_node = new RingNode(node_id, F_boundary);
    all_ring_node_.push_back(boundary_node);
    std::vector<RingNode*> target_nodes{boundary_node};
    node_map_[node_id] = boundary_node;
    node_id++;

    while(target_nodes.size()){
        std::vector<RingNode *> new_nodes;
        for(auto o_ring: target_nodes){
            std::vector<std::vector<GU::Point>> valid_rings = polygonOffset(o_ring->navline, swath_width_);
            if(valid_rings.size() < 1){
                valid_rings = polygonOffset(o_ring->navline, 0.5 * swath_width_);
            }
            // std::vector<std::vector<GU::Point>> valid_rings = po.inflatePolygon(o_ring, swath_width_);
            for(auto ring: valid_rings){
                // process obstacle
                if(!ignore_obstacle){
                    std::vector<std::vector<GU::Point>> valid_rings_nobs = processObstacles(ring, safe_obstacle);
                    for(auto& r:valid_rings_nobs){
                        RingNode* node = new RingNode(node_id, r);
                        all_ring_node_.push_back(node);
                        new_nodes.push_back(node);
                        o_ring->child.push_back(node);
                        node_map_[node_id] = node;
                        node_id++;
                    }
                }
                else{
                    RingNode* node = new RingNode(node_id, ring);
                    all_ring_node_.push_back(node);
                    new_nodes.push_back(node);
                    o_ring->child.push_back(node);
                    node_map_[node_id] = node;
                    node_id++;
                }
            }
        }
        plan_count++;
        if(plan_count > 1000){
            return false;
        }
        // std::cout << " iter " << plan_count << "valid ring num: " << new_rings.size() << std::endl;
        target_nodes = new_nodes;
    }

    // search path
    std::vector<GU::Point> trajectory;
    if(!planRingpath(boundary_node, trajectory, F_boundary, safe_obstacle, astar_search)){
        return false;
    }
    
    opt_trajectory.push_back(trajectory[0]);
    for(int i=0; i<trajectory.size()-2; ++i){
        double dx1, dy1, dx2, dy2, dx3, dy3, cross;
        dx1 = trajectory[i](0); dy1 = trajectory[i](1);
        dx2 = trajectory[i+1](0); dy2 = trajectory[i+1](1);
        dx3 = trajectory[i+2](0); dy3 = trajectory[i+2](1);
        cross = (dx2 - dx1)*(dy3 - dy2) - (dy2 - dy1)*(dx3 - dx2);
        if(std::fabs(cross) > 1e-3){
            opt_trajectory.push_back(trajectory[i+1]);
        }
    }
    opt_trajectory.push_back(trajectory[trajectory.size() - 1]);
    
    clock_t time_end_cpu111 = clock();
    double planning_duration = (double)(time_end_cpu111 - time_begin_cpu) / CLOCKS_PER_SEC;
    std::cout<<"planning duration: "<<planning_duration<<std::endl;

#ifdef FC_DEBUG
    cv::Mat map_with_path = map_.clone();
    map_with_path.setTo(cv::Scalar(255));
    int mx, my;
    for(int i=0; i<opt_trajectory.size()-1; ++i){
        worldToMap(opt_trajectory[i](0), opt_trajectory[i](1), mx, my);
        cv::Point start_point(mx, my);
        worldToMap(opt_trajectory[i+1](0), opt_trajectory[i+1](1), mx, my);
        cv::Point end_point(mx, my);
        cv::line(map_with_path, start_point, end_point, cv::Scalar(0), 1.5);
    }
    cv::imwrite("map_with_path.jpg", map_with_path);
#endif

    return true;
}
