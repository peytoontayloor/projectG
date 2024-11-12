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

                // std::map<std::pair<double, double>, long signed int> ::iterator it;

                // // for (it = map.begin(); it != map.end(); it++)
                // {
                //     std::cout << "(" << it->first.first  << ", "  // string (key)
                //             << it->first.second  << " ) "  // string (key)
                //             << ':'
                //             << it->second   // string's value 
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
                //     std::cout << "(" << it->first.first  << ", "  // string (key)
                //              << it->first.second  << " ) "  // string (key)
                //             << ':'
                //             << it->second   // string's value 
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


            std::vector<ompl::base::State *> getAdjacentVertices(PRM::Graph roadmap, std::map<std::pair<double, double>, long signed int> mapping, ompl::base::State * qnearSubState){
                
                double x_near = qnearSubState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double y_near = qnearSubState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                std::pair<double, double> coord_near (x_near, y_near);

                // std::cout << "x: " << x_near << std::endl;
                // std::cout << "y: " << y_near << std::endl;

                auto pos = mapping.find(coord_near);
                if (pos == mapping.end()){
                    // std::cout << "did not find" << "\n" << std::endl;
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
                    return getAdjacentVertices(r1RM, robot1mapping, qnear->as<ompl::base::CompoundState>()->components[0]);
                }

                if (robotId == 2){
                    return getAdjacentVertices(r2RM, robot2mapping, qnear->as<ompl::base::CompoundState>()->components[1]);
                }

                if (robotId == 3){
                    return getAdjacentVertices(r3RM, robot3mapping, qnear->as<ompl::base::CompoundState>()->components[2]);
                }

                if (robotId == 4){
                    return getAdjacentVertices(r4RM, robot4mapping, qnear->as<ompl::base::CompoundState>()->components[3]);
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
                ompl::base::State* nearState = info->allocState();
                ompl::base::State* randState = info->allocState();
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
                double angle = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i < neighbors.size(); i++){
                    ompl::base::State* temp = neighbors[i];
                    double x = temp->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = temp->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    double d = euclideanDistance(nearX, nearY, x, y);
                    double n = euclideanDistance(randX, randY, x, y);
                    double m = euclideanDistance(nearX, nearY, randX, randY); 
                    double numerator = d * d + m * m - n * n;
                    double denominator = 2 * d * m;
                    double temp_angle = acos(numerator / denominator);
                    if (temp_angle < angle){
                        info->copyState(qnew, temp);
                        angle = temp_angle;
                    }
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

            std::vector<int> robotPathCollisionChecking(std::vector<std::pair<ompl::base::State *, ompl::base::State *>> robotMovements)
            {
                std::pair<ompl::base::State *, ompl::base::State *> r1movement = robotMovements[0];
                std::pair<ompl::base::State *, ompl::base::State *> r2movement = robotMovements[1];
                std::pair<ompl::base::State *, ompl::base::State *> r3movement = robotMovements[2];
                std::pair<ompl::base::State *, ompl::base::State *> r4movement = robotMovements[3];
                
                double r1XQnear = r1movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r1YQnear = r1movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r1XQnew = r1movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r1YQnew = r1movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r2XQnear = r2movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r2YQnear = r2movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r2XQnew = r2movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r2YQnew = r2movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r3XQnear = r3movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r3YQnear = r3movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r3XQnew = r3movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r3YQnew = r3movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r4XQnear = r4movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r4YQnear = r4movement.first -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double r4XQnew = r4movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double r4YQnew = r4movement.second -> as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                Line r1move;
                r1move.x1 = r1XQnear;
                r1move.y1 = r1YQnear;
                r1move.x2 = r1XQnew;
                r1move.y2 = r2YQnew;
                
                Line r2move;
                r2move.x1 = r2XQnear;
                r2move.y1 = r2YQnear;
                r2move.x2 = r2XQnew;
                r2move.y2 = r2YQnew;
                
                Line r3move;
                r3move.x1 = r3XQnear;
                r3move.y1 = r3YQnear;
                r3move.x2 = r3XQnew;
                r3move.y2 = r3YQnew;
                
                Line r4move;
                r4move.x1 = r4XQnear;
                r4move.y1 = r4YQnear;
                r4move.x2 = r4XQnew;
                r4move.y2 = r4YQnew;
                
                std::vector<int> res;
                bool robot1Collision = linePointIntersection(r1XQnew, r1YQnew, r2move) || linePointIntersection(r1XQnew, r1YQnew, r3move) || linePointIntersection(r1XQnew, r1YQnew, r4move);
                if (robot1Collision){
                    res.push_back(1);
                }
                else{
                    res.push_back(0);
                }

                bool robot2Collision = linePointIntersection(r2XQnew, r2YQnew, r1move) || linePointIntersection(r2XQnew, r2YQnew, r3move) || linePointIntersection(r2XQnew, r2YQnew, r4move);
                if (robot2Collision){
                    res.push_back(1);
                }
                else{
                    res.push_back(0);
                }

                bool robot3Collision = linePointIntersection(r3XQnew, r3YQnew, r1move) || linePointIntersection(r3XQnew, r3YQnew, r2move) || linePointIntersection(r3XQnew, r3YQnew, r4move);
                if (robot3Collision){
                    res.push_back(1);
                }
                else{
                    res.push_back(0);
                }

                bool robot4Collision = linePointIntersection(r4XQnew, r4YQnew, r1move) || linePointIntersection(r4XQnew, r4YQnew, r2move) || linePointIntersection(r4XQnew, r4YQnew, r3move);
                if (robot4Collision){
                    res.push_back(1);
                }
                else{
                    res.push_back(0);
                }
                return res;
            }

            std::set<ompl::base::State *> explored;

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
            ompl::base::State * customCompositeSampler(ompl::base::StateSpacePtr space);

            // Stores mapping of nodes to a vector of nodes
            std::map<std::vector<std::pair<double, double>>, std::vector<std::vector<std::pair<double, double>>>> adjList;

            // Stores mapping of nodes to their indegrees
            std::map<std::vector<std::pair<double, double>>, int> indegree;

            // Turns a state into a vector of four coordinates and initializes the indegree to 0
            void addVertex(ompl::base::State * a){

                const ompl::base::CompoundState * a_compoundstate = a->as<ompl::base::CompoundState>();

                // Create vector of four (x, y) pairs 
                std::vector<std::pair<double, double>> coords;
                for (int i = 0; i < 4; ++i){
                    double rix_coord = a_compoundstate->as<ompl::base::RealVectorStateSpace::StateType>(i)->values[0];
                    double riy_coord = a_compoundstate->as<ompl::base::RealVectorStateSpace::StateType>(i)->values[1];
                    
                    std::pair<double, double> coord (rix_coord, riy_coord);
                    coords.push_back(coord);
                }

                // Create node v and add indegree
                indegree[coords] = 0;

            }

            bool vertexInGraph(std::vector<std::pair<double, double>> v){
                std::map<std::vector<std::pair<double, double>>, std::vector<std::vector<std::pair<double, double>>>>::iterator it = adjList.find(v);
                if (it == adjList.end()){
                    return false;
                }
                return true;
            }

            void setIndegrees(){
                std::deque<std::vector<std::pair<double, double>>> deque;
                std::set<std::vector<std::pair<double, double>>> visited;

                std::vector<std::pair<double, double>> firstNode = indegree.begin()->first;
                visited.insert(firstNode);
                deque.push_back(firstNode);

                // Running BFS to set indegrees of nodes correctly
                while (!deque.empty()){
                    std::vector<std::pair<double, double>> node = deque.front();
                    deque.pop_front();
                    
                    auto adjIt = adjList.find(node);
                    if (adjIt != adjList.end()){
                        std::vector<std::vector<std::pair<double, double>>> nbrs = adjIt->second;
                        for (size_t i = 0; i < nbrs.size(); ++i){
                            if (visited.find(nbrs[i]) == visited.end()){
                                visited.insert(nbrs[i]);
                                auto indegreeIt = indegree.find(node);
                                if (indegreeIt!= indegree.end()){
                                    indegree[nbrs[i]] = indegree[nbrs[i]] + 1;
                                }
                                deque.push_back(nbrs[i]);
                            }
                        }
                    }

                }
            }

            bool graphSearch(){

                std::vector<std::vector<std::pair<double, double>>> verts;
                std::set<std::vector<std::pair<double, double>>> visited;
                
                std::deque<std::vector<std::pair<double, double>>> deque;

                std::map<std::vector<std::pair<double, double>>, int>::iterator it;
                for (it = indegree.begin(); it != indegree.end(); it++)
                {
                    if (it->second == 0){
                        deque.push_back(it->first);
                        visited.insert(it->first);
                    }
                    verts.push_back(it->first);
                }

                while (!deque.empty()){
                    std::vector<std::pair<double, double>> node = deque.front();

                    // Double checking here that node is in verts, it should be
                    std::vector<std::vector<std::pair<double, double>>>::iterator vertsIt = std::find(verts.begin(), verts.end(), node);
                    if (vertsIt != verts.end()){
                        verts.erase(vertsIt);
                    }

                    deque.pop_front();

                    // Running bfs 
                    auto adjIt = adjList.find(node);
                    if (adjIt != adjList.end()){
                        std::vector<std::vector<std::pair<double, double>>> nbrs = adjIt->second;

                        // iterate through neighbors
                        for (size_t i = 0; i < nbrs.size(); ++i){

                            // Ensure nbr[i] is not visited yet
                            if (visited.find(nbrs[i]) == visited.end()){
                                visited.insert(nbrs[i]);

                                // Decrement indegrees
                                auto indegreeIt = indegree.find(node);
                                if (indegreeIt!= indegree.end()){
                                    indegree[nbrs[i]] = indegree[nbrs[i]] - 1;

                                    // If indegrees becomes 0, add to queue
                                    if (indegree[nbrs[i]] == 0){
                                        deque.push_back(nbrs[i]);
                                    }
                                }
                            }
                        }
                    }
                }
                if (verts.empty()){
                    return true;
                }
                else{
                    return false;
                }

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
