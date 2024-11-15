/* Author: Ioan Sucan */
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/State.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <utility>
#include <deque>
#include <ompl/base/SpaceInformation.h>
 #include <boost/graph/graph_traits.hpp>
 #include <boost/graph/adjacency_list.hpp>
 #include <boost/pending/disjoint_sets.hpp>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/geometric/SimpleSetup.h>

// Definition of a line segment between (x1, y1) and (x2, y2)
struct Line
{
    double x1, y1;
    double x2, y2;
};


namespace ompl
{
    namespace geometric
    {
        
        /** \brief Rapidly-exploring Random Trees */
        class dRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            dRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~dRRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

            PRM::Graph r1RM;
            PRM::Graph r2RM;
            PRM::Graph r3RM;
            PRM::Graph r4RM;

            void setRobotPRMs(PRM::Graph r1RM_param, PRM::Graph r2RM_param, PRM::Graph r3RM_param, PRM::Graph r4RM_param)
            {
                r1RM = r1RM_param;
                r2RM = r2RM_param;
                r3RM = r3RM_param;
                r4RM = r4RM_param;

            }

            std::vector<ompl::base::State *> robot1;
            std::vector<ompl::base::State *> robot2;
            std::vector<ompl::base::State *> robot3;
            std::vector<ompl::base::State *> robot4;

            std::map<std::pair<double, double>, long signed int> robot1mapping;
            std::map<std::pair<double, double>, long signed int> robot2mapping;
            std::map<std::pair<double, double>, long signed int> robot3mapping;
            std::map<std::pair<double, double>, long signed int> robot4mapping;

            ompl::base::SpaceInformationPtr spaceInfo1;
            ompl::base::SpaceInformationPtr spaceInfo2;
            ompl::base::SpaceInformationPtr spaceInfo3;
            ompl::base::SpaceInformationPtr spaceInfo4;

            // Vector to keep track of our explored space (ensures qrand doesn't sample from explored)
            std::vector<ompl::base::State *>  explored;

            std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> createPRMNodes(PRM::Graph roadmap){

                // Maps vertices to state - name of property is og::PRM::vertex_state_t() 
                auto stateMap = boost::get(PRM::vertex_state_t(), roadmap);

                // Iterate through graph nodes
                PRM::Graph::vertex_iterator v, vend;

                // Extract states into vector
                std::vector<ompl::base::State *> states;

                // Set up a mapping of state's x, y coordinates to a vertex descriptor, later used to find adjacent vertices
                std::map<std::pair<double, double>, long signed int> map;

                // Iterate through all vertices 
                for (boost::tie(v, vend) = boost::vertices(roadmap); v != vend; ++v) {

                    ompl::base::State *state = boost::get(stateMap, *v);

                    // TODO: potentially reconsider pushing a pointer to a state, it works but is not good practice
                    states.push_back(state);
                    
                    // std::cout << "v*: " << *v << std::endl;

                    // Create tuple of coordinates to use as key in mapping (instead of pointer to state)
                    double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    std::pair<double, double> coord (x, y);

                    map[coord] = *v;

                    // std::cout << "x: " << x << std::endl;
                    // std::cout << "y: " << y << "\n" << std::endl;
                }

                // Make tuple of vector of states and mapping to return
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> result(states, map);

                // std::cout << "---------" << std::endl;

                // for (auto it = map.begin(); it != map.end(); it++)
                // {
                //     std::cout << "(" << it->first.first  << ", " // x coordinate
                //             << it->first.second  << " ) "        // y coordinate
                //             << ':'
                //             << it->second                        // vertex descriptor
                //             << std::endl;
                // }
                return result;
            }

            void setRobotNodes()
            {
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> resultR1 = createPRMNodes(r1RM);
                robot1 = resultR1.first;
                robot1mapping = resultR1.second;

                // std::map<std::pair<double, double>, long signed int> ::iterator it;
                // for (it = robot1mapping.begin(); it != robot1mapping.end(); it++)
                // {
                //     std::cout << "(" << it->first.first  << ", "  // x coordinate
                //              << it->first.second  << " ) "       // y coordinate
                //             << ':'
                //             << it->second                         // vertex descriptor
                //             << std::endl;
                // }
                
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>>  resultR2 = createPRMNodes(r2RM);
                robot2 = resultR2.first;
                robot2mapping = resultR2.second;
                
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>>  resultR3 = createPRMNodes(r3RM);
                robot3 = resultR3.first;
                robot3mapping = resultR3.second;

                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>>  resultR4 = createPRMNodes(r4RM);
                robot4 = resultR4.first;
                robot4mapping = resultR4.second;
            }

            void setIndivSpaceInfo(ompl::geometric::SimpleSetupPtr si1, ompl::geometric::SimpleSetupPtr si2, ompl::geometric::SimpleSetupPtr si3, ompl::geometric::SimpleSetupPtr si4)
            {
                spaceInfo1 = si1->getSpaceInformation();
                spaceInfo2 = si2->getSpaceInformation();
                spaceInfo3 = si3->getSpaceInformation();
                spaceInfo4 = si4->getSpaceInformation();
            }


            std::vector<ompl::base::State *> getAdjacentVertices(PRM::Graph roadmap, std::map<std::pair<double, double>, long signed int> mapping, ompl::base::State * qnear, int robotId){

                double x_near = 0.0;
                double y_near = 0.0;
                ompl::base::CompoundState * qnearCompoundState = qnear->as<ompl::base::CompoundState>();
                if (robotId == 1)
                {
                    x_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
                    y_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
                }
                if (robotId == 2)
                {
                    x_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
                    y_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];
                }
                if (robotId == 3)
                {
                    x_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
                    y_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];   
                }
                if (robotId == 4){
                    x_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[0];
                    y_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[1];
                }
                
                std::pair<double, double> coord_near (x_near, y_near);

                // std::cout << "x: " << x_near << std::endl;
                // std::cout << "y: " << y_near << "\n" << std::endl;

                auto pos = mapping.find(coord_near);
                if (pos == mapping.end()){
                    std::cout << "did not find" << "\n" << std::endl;
                    std::vector<ompl::base::State *> result = {};
                    return result;
                }
                else{

                    long signed int v = pos->second;

                    auto stateMap = boost::get(PRM::vertex_state_t(), roadmap);

                    PRM::Graph::adjacency_iterator ai, a_end; 
                    std::vector<ompl::base::State *> states;

                    // std::cout << "---------" << std::endl;
                    // std::cout << "neighbors of " << std::endl;
                    // std::cout << v << std::endl;
                    // std::cout << "---------" << std::endl;

                    for (boost::tie(ai, a_end) = boost::adjacent_vertices(v, roadmap); ai != a_end; ai++) { 
                        ompl::base::State *state = boost::get(stateMap, *ai);
                        states.push_back(state);

                        double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                        double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                        // std::cout << "x: " << x << std::endl;
                        // std::cout << "y: " << y << "\n" << std::endl;
                    }
                    // std::cout << "---------" << "\n" << std::endl;
                    return states;
                }

            }

            std::vector<ompl::base::State *> neighbors(ompl::base::State * qnear, int robotId){

                if (robotId == 1){
                    return getAdjacentVertices(r1RM, robot1mapping, qnear, robotId);
                }

                if (robotId == 2){
                    return getAdjacentVertices(r2RM, robot2mapping, qnear, robotId);
                }

                if (robotId == 3){
                    return getAdjacentVertices(r3RM, robot3mapping, qnear, robotId);
                }

                if (robotId == 4){
                    return getAdjacentVertices(r4RM, robot4mapping, qnear, robotId);
                }

            }

            double euclideanDistance(double xCoordA, double yCoordA, double xCoordB, double yCoordB)
            {
                return sqrt(std::pow(xCoordA - xCoordB, 2) + std::pow(yCoordA - yCoordB, 2));
            }


            ompl::base::State* oracle(ompl::base::State* qnear, ompl::base::State* qrand, std::vector<ompl::base::State *> neighbors, int rNum, ompl::base::SpaceInformationPtr info)
            {
                // This will return a state, qnew, such that angle is minimized (following algorithm provided in paper)
                
                // Do not need to make a newmotion here, will be done at the end of all robot's qnews being generated

                // TODO: ensure that the qnew picked is not in "explored states" --> implement this later 

                // Extract the individual states from the composite ones
                //ompl::base::State* nearState = info->allocState();
                //ompl::base::State* randState = info->allocState();
                ompl::base::State* nearState;
                ompl::base::State* randState;
                if (rNum == 1)
                {
                    nearState = qnear->as<ompl::base::CompoundState>()->components[0];
                    randState = qrand->as<ompl::base::CompoundState>()->components[0];
                }
                if (rNum == 2)
                {
                    nearState = qnear->as<ompl::base::CompoundState>()->components[1];
                    randState = qrand->as<ompl::base::CompoundState>()->components[1];
                }
                if (rNum == 3)
                {
                    nearState = qnear->as<ompl::base::CompoundState>()->components[2];
                    randState = qrand->as<ompl::base::CompoundState>()->components[2];
                }
                if (rNum == 4)
                {
                    nearState = qnear->as<ompl::base::CompoundState>()->components[3];
                    randState = qrand->as<ompl::base::CompoundState>()->components[3];
                }

                double nearX = nearState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double nearY = nearState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double randX = randState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double randY = randState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                // find q new by minimizing angle between line from q_near--q_rand and q_new---q_rand
                
                ompl::base::State *qnew = info->allocState();
                info->copyState(qnew, neighbors[0]);

                double x_ = neighbors[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double y_ = neighbors[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                double d_ = euclideanDistance(nearX, nearY, x_, y_);
                double n_ = euclideanDistance(randX, randY, x_, y_);
                double m_ = euclideanDistance(nearX, nearY, randX, randY); 
                double numerator = d_ * d_ + m_ * m_ - n_ * n_;
                double denominator = 2 * d_ * m_;
                double temp_angle = acos(numerator / denominator);

                // std::cout << "neighbors size: " << neighbors.size() << std::endl;
                double angle = temp_angle;
                ompl::base::State* temp = info->allocState();
                for (size_t i = 1; i < neighbors.size(); i++){
                    info->copyState(temp, neighbors[i]);
                    
                    double x = temp->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = temp->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    double d = euclideanDistance(nearX, nearY, x, y);
                    double n = euclideanDistance(randX, randY, x, y);
                    double m = euclideanDistance(nearX, nearY, randX, randY); 
                    double numerator = d * d + m * m - n * n;
                    double denominator = 2 * d * m;

                    // TODO: make sure in valid range!
                    if (denominator == 0)
                    {
                        continue;
                    }
                    double temp_angle = acos(numerator / denominator);

                    if ((temp_angle > 0) && (temp_angle < angle))
                    {
                        info->copyState(qnew, temp);
                        angle = temp_angle;
                    }
                }
                info->freeState(temp);

                if (angle == std::numeric_limits<double>::infinity()){
                    return nullptr;
                }

                return qnew;

            }

            bool linePointIntersection(double x, double y, Line line)
            {
                if (euclideanDistance(line.x1, line.y1, line.x2, line.y2) == euclideanDistance(x, y, line.x1, line.y1) + euclideanDistance(x, y, line.x2, line.y2)){
                    return true;
                }
                return false;
            }

            // std::vector<ompl::base::State *> nearestN(ompl::base::State* qnear);

            // ompl::base::State* smallestDist(ompl::base::State* source, std::vector<ompl::base::State *> robotStates);

            double customDistanceFunction(ompl::base::State * a, ompl::base::State * b){

                // Cast to compound state from ompl::base::State
                const ompl::base::CompoundState * a_cstate = a->as<ompl::base::CompoundState>();
                const ompl::base::CompoundState * b_cstate = b->as<ompl::base::CompoundState>();

                // Computing Euclidean distance of r1 
                double a_r1x_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
                double a_r1y_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];

                double b_r1x_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
                double b_r1y_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];

                double r1Distance = euclideanDistance(a_r1x_coord, a_r1y_coord, b_r1x_coord, b_r1y_coord);

                // Computing Euclidean distance of r2
                double a_r2x_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
                double a_r2y_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];

                double b_r2x_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
                double b_r2y_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];

                double r2Distance = euclideanDistance(a_r2x_coord, a_r2y_coord, b_r2x_coord, b_r2y_coord);

                // Computing Euclidean distance of r3
                double a_r3x_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
                double a_r3y_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];

                double b_r3x_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
                double b_r3y_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];

                double r3Distance = euclideanDistance(a_r3x_coord, a_r3y_coord, b_r3x_coord, b_r3y_coord);

                // Computing Euclidean distance of r4
                double a_r4x_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[0];
                double a_r4y_coord = a_cstate->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[1];

                double b_r4x_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[0];
                double b_r4y_coord = b_cstate->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[1];

                double r4Distance = euclideanDistance(a_r4x_coord, a_r4y_coord, b_r4x_coord, b_r4y_coord);
                
                // Returns sum of euclidean distances
                return r1Distance + r2Distance + r3Distance + r4Distance;
            }

            

            // TODO: keeping new 'sampler' here, not sure if this is the best practice, might move
            ompl::base::State * customCompositeSampler(ompl::base::StateSpacePtr space, ompl::base::State *goal);

            // Stores mapping of nodes to a vector of nodes
            std::map<std::vector<std::pair<double, double>>, std::vector<std::vector<std::pair<double, double>>>> adjList;

            // Stores mapping of nodes to their indegrees
            // We don't want this global bc changes each time we try to add qnew 
            //std::map<std::vector<std::pair<double, double>>, int> indegree;

            bool localConnector(ompl::base::State *cmpdnear, ompl::base::State *cmpdnew)
            {
                std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> graph;
                // First, extract each robots qnear and qnew and then add them to our graph
                // For 4 robots, a start and goal point each, we have 8 keys to add to the map

                // TODO: might need to free states after each loop? But would that free the state completely or just from this assignment?

                ompl::base::State *qnear;
                ompl::base::State *qnew;
                //std::cout << "NEW LOCAL CHECK:" << std::endl;
                for(int i = 0; i < 4; i++)
                {
                    qnear = cmpdnear->as<ompl::base::CompoundState>()->components[i];
                    // qnew = cmpdnew->as<ompl::base::CompoundState>()->components[i];

                    double nearX = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double nearY = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    // double newX = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    // double newY = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    // std::cout << "NEAR:" << std::endl;
                    // std::cout << "(" << nearX << ", " << nearY << ")" << std::endl;
                    // std::cout << "NEW:" << std::endl;
                    // std::cout << "(" << newX << ", " << newY << ")" << std::endl;

                    std::pair<double, double> nearCoord (nearX, nearY);
                    // std::pair<double, double> newCoord (newX, newY);

                    // add all the q nears
                    graph[nearCoord];

                    // // If near is in the graph already, add qnew to its list of what it points to
                    // if(graph.find(nearCoord) != graph.end())
                    // {
                    //     graph[nearCoord].push_back(newCoord);
                    //     if(graph.find(newCoord) == graph.end())
                    //     {
                    //         graph[newCoord];
                    //     } 
                    // }
                    // else
                    // {
                    //     // If qnew is in the graph, keep looping, if not add it in, pointing to: []
                    //     if(graph.find(newCoord) != graph.end())
                    //     {
                    //         continue;
                    //     }
                    //     else
                    //     {
                    //         graph[newCoord];
                    //     }
                    // }

                }

                for (int i = 0 ; i < 4; ++i){

                    qnear = cmpdnear->as<ompl::base::CompoundState>()->components[i];
                    qnew = cmpdnew->as<ompl::base::CompoundState>()->components[i];

                    double nearX = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double nearY = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    double newX = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double newY = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    std::pair<double, double> newCoord (newX, newY);
                    std::pair<double, double> nearCoord (nearX, nearY);

                    if (graph.find(newCoord) != graph.end())
                    {
                        graph[newCoord].push_back(nearCoord);
                    }

                }
                
                // std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> ::iterator it;
                // for (it = graph.begin(); it != graph.end(); it++)
                // {
                //     std::cout << "(" << it->first.first  << ", " << it->first.second << ")" << std::endl;
                //     std::cout << "[" << std::endl;
                //     std::cout << it->second.size() << std::endl;
                //     for (size_t i = 0; i < it->second.size(); i++){
                //         std::cout << it->second[i].first << ", " << it->second[i].second << std::endl;
                //     }
                //     std::cout << "] \n" << std::endl;
                // }
                

                // Now that we have graph storing which nodes point to which, we need to create a map of the indegrees of each node:
                std::map<std::pair<double, double>, int> inDegree;
                //std::vector<std::pair<double, double>> vertices;
                std::map<std::pair<double, double>, bool> vertices;

                // Loop through our main graph and initialize every node's inDegree to 0
                // Also, create a map holding if the nodes have been visited or not
                // Keys of our graph should have all the nodes based on how we initialized it
                //std::cout << "NEW" << std::endl;
                for (auto i = graph.begin(); i != graph.end(); i++)
                {
                    //std::cout << "NODE: " << i->first.first <<  ", " << i->first.second << std::endl;
                    //i->first is key
                    //i->second is value
                    inDegree[(i->first)] = 0;
                    //vertices.push_back(i->first);
                    vertices[(i->first)] = false;
                }

                // Now, loop again, updating the indegrees of each node
                for (auto i = graph.begin(); i != graph.end(); i++)
                {
                    // loop through the nodes it points to and update their indegrees
                    for(size_t j = 0; j < i->second.size(); j++)
                    {
                        inDegree[i->second[j]] += 1;
                    }
                }

                //std::deque<std::pair<double, double>> queue;
                std::vector<std::pair<double, double>> queue;
                // Now, get our queue which should be a vector of only the vertices with indegree = 0
                for (auto i = inDegree.begin(); i != inDegree.end(); i++)
                {
                    if(i->second == 0)
                    {
                        queue.push_back(i->first);
                    }
                }

                // When working with our queue, access the last element and pop the last element (pop_back() and back())
                while(!(queue.empty()))
                {
                    // Pop the back element (doing this bc easy methods in c++ (pop_back, back(), push_back))
                    //std::pair<double, double> vertex = queue.front();
                    //queue.pop_front();
                    std::pair<double, double> vertex = queue.back();
                    queue.pop_back();

                    // Remove vertex from list of remaining vertices
                    // UPDATE: made a map tracking visited vertices, mark as true if visited here:
                    vertices[vertex] = true;

                    // Get the neighbors of the removed vertex, and decrement their indegree counts, adding to queue if at 0
                    for(size_t i = 0; i < graph[vertex].size(); i++)
                    {
                        inDegree[graph[vertex][i]] -= 1;
                        if (inDegree[graph[vertex][i]] == 0)
                        {
                            queue.push_back(graph[vertex][i]);
                        }
                    }
                }

                // Once queue is empty, if one of our vertices is false, then there is a cycle somewhere/cannot assign priorities to the robots for valid movement
                for (auto i = vertices.begin(); i != vertices.end(); i++)
                {
                    if (i->second == false)
                    {
                        std::cout << "CYCLE" << std::endl;
                        return false;
                    }
                }
                return true;

            }
         

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }


            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}
