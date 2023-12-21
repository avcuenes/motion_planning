#include "rrtstar.hpp"
RRTStar::RRTStar(){

}

bool RRTStar::isCollisionFree(NodeRRT* from, NodeRRT* to) {
    // Euclidean distance for collision check
        double distance = std::sqrt(std::pow(to->x - from->x, 2) + std::pow(to->y - from->y, 2));

        // Check for collisions along the path
        for (double t = 0.0; t <= 1.0; t += 0.1) {
            double intermediateX = from->x + t * (to->x - from->x);
            double intermediateY = from->y + t * (to->y - from->y);

            // Check for collisions with obstacles
            if (isPointInCollision(intermediateX, intermediateY)) {
                return false;  // Collision detected
            }
        }

        return true;  // Path is collision-free
    return true;
}

bool RRTStar::isPointInCollision(double x, double y) {
    // Check for collisions with obstacles
    for (const auto& obstacle : obstacles) {
        double obstacleRadius = obstacle.r;  // Adjust this value based on your obstacle size
        double distanceToObstacle = std::sqrt(std::pow(x - obstacle.x, 2) + std::pow(y - obstacle.y, 2));
        
        if (distanceToObstacle < obstacleRadius) {
            return true;  // Point is in collision with an obstacle
        }
    }

    return false;  // Point is not in collision with any obstacle
}


NodeRRT* RRTStar::generateRandomNode() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> xDist(minX, maxX);
    std::uniform_real_distribution<> yDist(minY, maxY);

    return new NodeRRT(xDist(gen), yDist(gen),static_cast<double>(rand()) / RAND_MAX * 2 * M_PI - M_PI,
                    (static_cast<double>(rand()) / RAND_MAX) * MAX_SPEED,
                    (static_cast<double>(rand()) / RAND_MAX) * 2 * MAX_OMEGA - MAX_OMEGA);
}

NodeRRT* RRTStar::findNearestNode(const std::vector<NodeRRT*>& nodes, NodeRRT* target) {
    NodeRRT* nearest = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& node : nodes) {
        double distance = std::sqrt(std::pow(target->x - node->x, 2) + std::pow(target->y - node->y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            nearest = node;
        }
    }

    return nearest;
}

NodeRRT* RRTStar::steer(NodeRRT* from, NodeRRT* to) {
    double angleDiff = atan2(sin(to->theta - from->theta), cos(to->theta - from->theta));
    double omega = std::max(-MAX_OMEGA, std::min(MAX_OMEGA, angleDiff / DT));
    return new NodeRRT(from->x + from->speed * cos(from->theta + omega * DT),
                from->y + from->speed * sin(from->theta + omega * DT),
                from->theta + omega * DT, from->speed, omega);
}

void RRTStar::rewire(NodeRRT* newNode, const std::vector<NodeRRT*>& nearNodes) {
    for (auto& nearNode : nearNodes) {
        double cost = nearNode->cost + std::sqrt(std::pow(newNode->x - nearNode->x, 2) + std::pow(newNode->y - nearNode->y, 2));

        if (cost < newNode->cost && isCollisionFree(nearNode, newNode)) {
            newNode->parent = nearNode;
            newNode->cost = cost;

            // Update the cost of all children in the subtree rooted at the new node
            updateCost(newNode);
        }
    }
}

void RRTStar::updateCost(NodeRRT* node) {
    for (auto& child : node->children) {
        child->cost = node->cost + std::sqrt(std::pow(child->x - node->x, 2) + std::pow(child->y - node->y, 2));
        updateCost(child);
    }
}


std::vector<NodeRRT*> RRTStar::RRTStarAlgorithm() {
    std::vector<NodeRRT*> nodes;
    NodeRRT* root = new NodeRRT(startX, startY,1,1,1);
    root->cost = 0.0;
    root->parent  = nullptr;
    nodes.push_back(root);

    for (int iter = 0; iter < maxIterations; ++iter) {
        NodeRRT* randomNode = generateRandomNode();
        NodeRRT* nearestNode = findNearestNode(nodes, randomNode);
        NodeRRT* newNode = steer(nearestNode, randomNode);

        if (isCollisionFree(nearestNode, newNode)) {
            std::vector<NodeRRT*> nearNodes = findNearNodes(nodes, newNode);
            newNode->parent = nearestNode;
            newNode->cost = nearestNode->cost + std::sqrt(std::pow(newNode->x - nearestNode->x, 2) + std::pow(newNode->y - nearestNode->y, 2));

            nodes.push_back(newNode);
            rewire(newNode, nearNodes);
        }

        
    }

   // Connect the goal to the nearest node in the final tree
    NodeRRT* goalNode = new NodeRRT(goalX, goalY, 1, 1, 1);
    NodeRRT* nearestToGoal = findNearestNode(nodes, goalNode);
    NodeRRT* finalGoalNode = steer(nearestToGoal, goalNode);

    if (isCollisionFree(nearestToGoal, finalGoalNode)) {
        finalGoalNode->parent = nearestToGoal;
        finalGoalNode->cost = nearestToGoal->cost + std::sqrt(std::pow(finalGoalNode->x - nearestToGoal->x, 2) + std::pow(finalGoalNode->y - nearestToGoal->y, 2));

        nodes.push_back(finalGoalNode);
    }
      
    return nodes;
}


std::vector<NodeRRT*>  RRTStar::searchBestGoalNode(std::vector<NodeRRT*> nodes) {
    NodeRRT* bestGoalNode;
    std::vector<NodeRRT*> bestPath;
    double minCostToGoal = std::numeric_limits<double>::max();
    for (const auto& node : nodes) {
        double distanceToGoal = std::sqrt(std::pow(goalX - node->x, 2) + std::pow(goalY - node->y, 2));
        if (distanceToGoal < searchRadius && node->cost < minCostToGoal) {
            bestGoalNode = node;
            minCostToGoal = node->cost;
            bestPath.push_back(node);
        }
    }

    return bestPath;
}



std::vector<NodeRRT*> RRTStar::findNearNodes(const std::vector<NodeRRT*>& nodes, NodeRRT* target) {
    std::vector<NodeRRT*> nearNodes;

    for (const auto& node : nodes) {
        double distance = std::sqrt(std::pow(target->x - node->x, 2) + std::pow(target->y - node->y, 2));
        if (distance < searchRadius) {
            nearNodes.push_back(node);
        }
    }

    return nearNodes;
}
