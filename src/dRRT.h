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

            std::vector<PRM::Graph> roadmaps;

            void setRobotPRMs(std::vector<PRM::Graph> roadmaps_param)
            {
                roadmaps = roadmaps_param;

            }

            std::vector<std::vector<ompl::base::State *>> robotNodes;

            std::vector<std::map<std::pair<double, double>, long signed int>> robotmappings;

            std::vector<ompl::base::SpaceInformationPtr> spaceInfos;

            int noConnect = 0;

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

                    // Create tuple of coordinates to use as key in mapping (instead of pointer to state)
                    double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    std::pair<double, double> coord (x, y);

                    map[coord] = *v;

                }

                // Make tuple of vector of states and mapping to return
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> result(states, map);

                return result;
            }

            void setRobotNodes()
            {
                for (size_t i = 0; i < roadmaps.size(); ++i)
                {
                    std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> result = createPRMNodes(roadmaps[i]);
                    robotNodes.push_back(result.first);
                    robotmappings.push_back(result.second);
                }
            }

            void setIndivSpaceInfo(std::vector<ompl::geometric::SimpleSetupPtr> ssPtrs)
            {
                for (size_t i = 0; i < ssPtrs.size(); ++i)
                {
                    spaceInfos.push_back(ssPtrs[i]->getSpaceInformation());
                }
            }


            std::vector<ompl::base::State *> getAdjacentVertices(PRM::Graph roadmap, std::map<std::pair<double, double>, long signed int> mapping, ompl::base::State * qnear, int robotId){

                ompl::base::CompoundState * qnearCompoundState = qnear->as<ompl::base::CompoundState>();
                double x_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(robotId)->values[0];
                double y_near = qnearCompoundState->as<ompl::base::RealVectorStateSpace::StateType>(robotId)->values[1];
                
                std::pair<double, double> coord_near (x_near, y_near);

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

                    for (boost::tie(ai, a_end) = boost::adjacent_vertices(v, roadmap); ai != a_end; ai++) { 
                        ompl::base::State *state = boost::get(stateMap, *ai);
                        states.push_back(state);
                    }
                    return states;
                }

            }

            std::vector<std::vector<ompl::base::State *>> neighbors(ompl::base::State * qnear){

                std::vector<std::vector<ompl::base::State *>> res;
                for (size_t i = 0; i < roadmaps.size(); ++i)
                {
                    res.push_back(getAdjacentVertices(roadmaps[i], robotmappings[i], qnear, i));
                }
                
                return res;

            }

            double euclideanDistance(double xCoordA, double yCoordA, double xCoordB, double yCoordB)
            {
                return sqrt(std::pow(xCoordA - xCoordB, 2) + std::pow(yCoordA - yCoordB, 2));
            }


            ompl::base::State* oracle(ompl::base::State* qnear, ompl::base::State* qrand, std::vector<ompl::base::State *> neighbors, int rNum, ompl::base::SpaceInformationPtr info)
            {
                // This will return a state, qnew, such that angle is minimized (following algorithm provided in paper)

                // Assigned ot nullptr to fix comiler warnings, shouldn't ever not hit one of the conditions
                ompl::base::State* nearState = qnear->as<ompl::base::CompoundState>()->components[rNum];
                ompl::base::State* randState = qrand->as<ompl::base::CompoundState>()->components[rNum];

                double nearX = nearState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double nearY = nearState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double randX = randState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double randY = randState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                // find q new by minimizing angle between line from q_near--q_rand and q_new---q_rand
                
                ompl::base::State *qnew = info->allocState();
                double angle = std::numeric_limits<double>::infinity();
                
                //ompl::base::State* temp = info->allocState();
                for (size_t i = 1; i < neighbors.size(); i++){
                    
                    double x = neighbors[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = neighbors[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    double d = euclideanDistance(nearX, nearY, x, y);
                    double n = euclideanDistance(randX, randY, x, y);
                    double m = euclideanDistance(nearX, nearY, randX, randY); 
                    double numerator = d * d + m * m - n * n;
                    double denominator = 2 * d * m;

                    // make sure in valid range!
                    if (denominator == 0)
                    {
                        continue;
                    }
                    double temp_angle = acos(numerator / denominator);

                    // Is the > 0 check necessary? --> are negative angles okay? 
                    if ((temp_angle < angle) && (info->checkMotion(qnear, neighbors[i])))
                    {
                        info->copyState(qnew, neighbors[i]);
                        angle = temp_angle;
                    }
                }

                if (angle == std::numeric_limits<double>::infinity()){

                    info->freeState(qnew);
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

            bool localConnector(ompl::base::State *cmpdnear, ompl::base::State *cmpdnew)
            {
                std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> graph;
                // First, extract each robots qnear and qnew and then add them to our graph
                // For 4 robots, a start and goal point each, we have 8 keys to add to the map

                // might need to free states after each loop? But would that free the state completely or just from this assignment?

                ompl::base::State *qnear;
                ompl::base::State *qnew;
                for(int i = 0; i < 4; i++)
                {
                    qnear = cmpdnear->as<ompl::base::CompoundState>()->components[i];
                    qnew = cmpdnew->as<ompl::base::CompoundState>()->components[i];

                    double nearX = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double nearY = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    double newX = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double newY = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    std::pair<double, double> nearCoord (nearX, nearY);
                    std::pair<double, double> newCoord (newX, newY);

                    if (graph.find(nearCoord) == graph.end())
                    {
                        graph[nearCoord] = std::vector<std::pair<double, double>>();
                    }
                    if (graph.find(newCoord) == graph.end())
                    {
                        graph[newCoord] = std::vector<std::pair<double, double>>();
                    }

                    graph[nearCoord].push_back(newCoord);
                }
                

                // Now that we have graph storing which nodes point to which, we need to create a map of the indegrees of each node:
                std::map<std::pair<double, double>, int> inDegree;
                std::map<std::pair<double, double>, bool> vertices;

                // Loop through our main graph and initialize every node's inDegree to 0
                // Also, create a map holding if the nodes have been visited or not
                // Keys of our graph should have all the nodes based on how we initialized it
                //std::cout << "NEW" << std::endl;
                for (auto i = graph.begin(); i != graph.end(); i++)
                {
                    inDegree[(i->first)] = 0;
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
                        noConnect++;
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
            double goalBias_{.1};

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
