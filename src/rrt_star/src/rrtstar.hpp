#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

// Constants
const double MAX_SPEED = 1.0;
const double MAX_OMEGA = 1.0;
const double DT = 0.1;  // Time step

struct Obstacle {
    double x, y,r;
    Obstacle(double x_, double y_, double r_) : x(x_), y(y_), r(r_) {}
};


struct NodeRRT {
    double x, y;
    std::vector<NodeRRT*> children;
    NodeRRT* parent;
    double cost;

    double speed;
    double omega;
    double theta;

    NodeRRT(double x_, double y_, double theta_, double speed_, double omega_)
        : x(x_), y(y_), theta(theta_), speed(speed_), omega(omega_) {}
};

class RRTStar {
public:
    RRTStar();

    NodeRRT* generateRandomNode() ;

    NodeRRT* findNearestNode(const std::vector<NodeRRT*>& nodes, NodeRRT* target);

    NodeRRT* steer(NodeRRT* from, NodeRRT* to);

    bool isCollisionFree(NodeRRT* from, NodeRRT* to);

    bool isPointInCollision(double x, double y) ;

    void rewire(NodeRRT* newNode, const std::vector<NodeRRT*>& nearNodes);

    void updateCost(NodeRRT* node);

    std::vector<NodeRRT*> RRTStarAlgorithm() ;

    std::vector<NodeRRT*> findNearNodes(const std::vector<NodeRRT*>& nodes, NodeRRT* target) ;

    void setHomeandTargetPoint(double home_x, double home_y, double target_x, double target_y) 
        {startX = home_x; startY = home_y;goalX =target_x; goalY=target_y;};

    void setMapConstraint(double map_min_x, double map_min_y, double map_max_x, double map_max_y)
        {minX = map_min_x;minY = map_min_y;maxX = map_max_x,maxY =map_max_y;};

    void setStepSize(double step_size){stepSize = step_size;};

    void setMaxIterations(double max_iterations){maxIterations = max_iterations;};

    void addObstacle(double x, double y , double r) { obstacles.push_back(Obstacle(x, y, r));    }

    void clearObstacles() {obstacles.clear(); }

    std::vector<NodeRRT*>  searchBestGoalNode(std::vector<NodeRRT*> nodes);

    void GetPath(const std::vector<NodeRRT*>& nodes) {
        // Assuming the goal is the last node in the vector
        NodeRRT* current = nodes.back();

       for (int iter = 0; iter < nodes.size(); ++iter) {
            std::cout << "(" << current->x << ", " << current->y << ") ";
            current = current->parent;
        
        }

        std::cout << std::endl;
    }

private:
    double startX, startY, goalX, goalY;
    double minX, minY, maxX, maxY;
    double stepSize;
    int maxIterations;
    double searchRadius = 2.0; // Radius to find near nodes
    std::vector<Obstacle> obstacles;  // Vector to store obstacle positions

};

