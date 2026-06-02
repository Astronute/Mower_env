#include "astar.h"


AStar::AStar(){}

AStar::AStar(std::vector<std::vector<int>> gmap, const double &xcost, const double &ycost){
    initGridmap(gmap);
    x_move_cost_ = xcost;
    y_move_cost_ = ycost;
}

void AStar::initGridmap(const std::vector<std::vector<int>> &gmap){
    if(!initialized_){
        grid_map_ = gmap;
        width_ = grid_map_[0].size();
        height_ = grid_map_.size();
        
        state_node_map_.resize(height_);
        for(int i=0; i<height_; ++i){
            state_node_map_[i].resize(width_, nullptr);
        }
        initialized_ = true;
        std::cout << "A star planner initialized: " << std::endl;
    }
    else{
        std::cout << "This planner has already been initialized... doing nothing" << std::endl;
    }
}

double AStar::ComputeH(const std::shared_ptr<Node> &cur, const std::shared_ptr<Node> &end){
    return std::abs(cur->mx - end->mx)*x_move_cost_ + std::abs(cur->my - end->my)*y_move_cost_;
}

// move cost
double AStar::ComputeG(const std::shared_ptr<Node> &cur, const std::shared_ptr<Node> &neighbor){
    double dis = std::abs(cur->mx - neighbor->mx)*x_move_cost_*x_move_cost_ + std::abs(cur->my - neighbor->my)*y_move_cost_*y_move_cost_;
    return std::sqrt(dis);
}

bool AStar::CheckCollision(const unsigned int &mx, const unsigned int &my){
    // unsigned char cost = costmap_->getCost(mx, my);
    // if(costmap_->getCost(mx, my)>= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
    //     return true;
    // }
    if(grid_map_[my][mx]==1){
        return true;
    }
    return false;
}

bool AStar::CheckCollision(const unsigned int &mx, const unsigned int &my, const unsigned int &emx, const unsigned int &emy){
    // A to B
    if(std::abs(int(mx - emx))+std::abs(int(my - emy))==1){
        if(grid_map_[emy][emx]==1){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        if(grid_map_[emy][emx]==1 || grid_map_[emy][mx]==1 || grid_map_[my][emx]==1){
            return true;
        }
        else{
            return false;
        }
    }
}

bool AStar::CheckBoundary(const unsigned int &mx, const unsigned int &my){
    // if(mx>costmap_->getSizeInCellsX()||my>costmap_->getSizeInCellsY()){
    //     return true;
    // }
    if(mx>=width_ || my>=height_){
        return true;
    }
    return false;
}

bool AStar::findNearestPassable(const Eigen::Vector2i& point, Eigen::Vector2i& vpoint, unsigned int max_radius){
    if(!CheckCollision(point(0), point(1))){
        vpoint = point;
        return true;
    }

    std::queue<Eigen::Vector2i> q;
    std::set<std::pair<int, int>> visited;
    q.push(point);
    visited.insert({point(0), point(1)});

    const std::vector<Eigen::Vector2i> dirs = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
    int radius = 0;

    while(!q.empty() && radius <= max_radius){
        int len = q.size();
        radius++;
        for(int i=0; i<len; ++i){
            Eigen::Vector2i cur = q.front();
            q.pop();
            for(auto &dir: dirs){
                Eigen::Vector2i next = cur + dir;
                if(visited.count({next(0), next(1)})){
                    continue;
                }
                else{
                    visited.insert({next(0), next(1)});
                }
                if(!CheckCollision(next(0), next(1))){
                    vpoint = next;
                    return true;
                }
                q.push(next);
            }
        }
    }
    return false;
}

std::vector<std::shared_ptr<Node>> AStar::Expansion(const std::shared_ptr<Node> &node){
    std::vector<std::shared_ptr<Node>> surrNodes;
    for(int x=node->mx-1; x<=node->mx+1; ++x){
        for(int y=node->my-1; y<=node->my+1; ++y){
            if(!CheckBoundary(x, y) && !CheckCollision(node->mx, node->my, x, y) && (node->mx!=x||node->my!=y)){
                std::shared_ptr<Node> p = std::make_shared<Node>(x, y);
                surrNodes.push_back(p);
            }
        }
    }
    return surrNodes;
}

bool AStar::makePlan(const Eigen::Vector2i &start, const Eigen::Vector2i &goal, std::vector<Eigen::Vector2i> &path){
    if(!initialized_){
        std::cout << "This planner has not been initialized, please call initialize()" << std::endl;
        return false;
    }
    Reset();

    std::shared_ptr<Node> startNode = std::make_shared<Node>();
    std::shared_ptr<Node> goalNode = std::make_shared<Node>();

    Eigen::Vector2i valid_start, valid_goal;
    if(findNearestPassable(goal, valid_goal, 10)&&findNearestPassable(start, valid_start, 10)){
        goalNode->mx = valid_goal(0);
        goalNode->my = valid_goal(1);
        startNode->mx = valid_start(0);
        startNode->my = valid_start(1);
        startNode->g = 0;
        startNode->h = ComputeH(startNode, goalNode);
        startNode->f = startNode->g + startNode->h;
        startNode->parent = nullptr;
        state_node_map_[startNode->my][startNode->mx] = startNode;
        openlist.insert(startNode);
        startNode->status = Node::IN_OPENSET;
    }
    else{
        std::cout << " astar start/goal invalid " << std::endl;
        return false;
    }

    int t = 0;
    while(!openlist.empty()){
        t++;
        std::shared_ptr<Node> curNode = *openlist.begin();
        openlist.erase(openlist.begin());
        curNode->status = Node::IN_CLOSESET;
        if((curNode->mx==goalNode->mx) && (curNode->my==goalNode->my)){
            std::shared_ptr<Node> p = curNode;
            std::shared_ptr<Node> q = curNode->parent;
            while(q){
                p = q;
                q = q->parent;
                path.push_back(Eigen::Vector2i(p->mx, p->my));
            }
            reverse(path.begin(), path.end());
            return true;
        }
        std::vector<std::shared_ptr<Node>> surrondings = Expansion(curNode);
        for(auto neighbor: surrondings){
            std::shared_ptr<Node> nodeInstatemap = state_node_map_[neighbor->my][neighbor->mx];
            const double movecost = ComputeG(curNode, neighbor);
            const double neighbor_h = ComputeH(neighbor, goalNode);
            if(nodeInstatemap==nullptr){
                state_node_map_[neighbor->my][neighbor->mx] = neighbor;
                neighbor->status = Node::IN_OPENSET;
                neighbor->parent = curNode;
                neighbor->g = curNode->g + movecost;
                neighbor->h = neighbor_h;
                neighbor->f = neighbor->g + neighbor->h;
                openlist.insert(neighbor);
            }
            else if(nodeInstatemap->status==Node::IN_CLOSESET){
                continue;
            }
            else if(nodeInstatemap->status==Node::IN_OPENSET){
                if(curNode->g+movecost < nodeInstatemap->g){
                    nodeInstatemap->g = curNode->g + movecost;
                    nodeInstatemap->f = nodeInstatemap->g + nodeInstatemap->h;
                    nodeInstatemap->parent = curNode;
                }
            }
        }
    }
    std::cout << "A Star search failed total step:" << t << std::endl;
    return false;
}

void AStar::Reset(){
    openlist.clear();
    closelist.clear();
    for(int i = 0; i < height_; ++i){
        for(int j = 0; j < width_; ++j){
            state_node_map_[i][j] = nullptr;
        }
    }
}

