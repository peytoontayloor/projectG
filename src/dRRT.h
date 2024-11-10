/* Author: Ioan Sucan */
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/State.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <ompl/base/SpaceInformation.h>
 #include <boost/graph/graph_traits.hpp>
 #include <boost/graph/adjacency_list.hpp>
 #include <boost/pending/disjoint_sets.hpp>
#include <ompl/geometric/planners/prm/PRM.h>

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
                    
                    std::cout << "v*: " << *v << std::endl;

                    double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    std::pair<double, double> coord (x, y);

                    map[coord] = *v;

                    std::cout << "x: " << x << std::endl;
                    std::cout << "y: " << y << "\n" << std::endl;
                }

                // Make tuple of vector of states and mapping to return
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> result(states, map);

                std::cout << "---------" << std::endl;

                std::map<std::pair<double, double>, long signed int> ::iterator it;

                for (it = map.begin(); it != map.end(); it++)
                {
                    std::cout << "(" << it->first.first  << ", "  // string (key)
                            << it->first.second  << " ) "  // string (key)
                            << ':'
                            << it->second   // string's value 
                            << std::endl;
                }
                return result;
            }

            void setRobotNodes()
            {
                std::pair<std::vector<ompl::base::State *>, std::map<std::pair<double, double>, long signed int>> resultR1 = createPRMNodes(r1RM);
                robot1 = resultR1.first;
                robot1mapping = resultR1.second;

                std::map<std::pair<double, double>, long signed int> ::iterator it;
                for (it = robot1mapping.begin(); it != robot1mapping.end(); it++)
                {
                    std::cout << "(" << it->first.first  << ", "  // string (key)
                             << it->first.second  << " ) "  // string (key)
                            << ':'
                            << it->second   // string's value 
                            << std::endl;
                }
                
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


            std::vector<ompl::base::State *> getAdjacentVertices(PRM::Graph roadmap, std::map<std::pair<double, double>, long signed int> mapping, ompl::base::State * qnearSubState){

                // std::map<std::pair<double, double>, long signed int>::iterator it;

                // for (it = mapping.begin(); it != mapping.end(); it++)
                // {
                //     std::cout << "(" << it->first.first  << ", "  // string (key)
                //              << it->first.second  << " ) "  // string (key)
                //             << ':'
                //             << it->second   // string's value 
                //             << std::endl;
                // }

                double x_near = qnearSubState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double y_near = qnearSubState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                std::pair<double, double> coord_near (x_near, y_near);

                std::cout << "x: " << x_near << std::endl;
                std::cout << "y: " << y_near << std::endl;

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

                    ompl::base::State *input_state = boost::get(stateMap, v);
                    
                    std::cout << "---------" << std::endl;

                    std::cout << "neighbors of " << std::endl;

                    std::cout << v << std::endl;
                    double x_input = input_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    double y_input = input_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                    std::cout << "---------" << std::endl;

                    std::cout << "x: " << x_input << std::endl;
                    std::cout << "y: " << y_input << "\n" << std::endl;

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

            std::set<ompl::base::State *> explored;

            // std::vector<ompl::base::State *> nearestN(ompl::base::State* qnear);

            // ompl::base::State* smallestDist(ompl::base::State* source, std::vector<ompl::base::State *> robotStates);

            double euclideanDistance(double xCoordA, double yCoordA, double xCoordB, double yCoordB)
            {
                return sqrt(std::pow(xCoordA - xCoordB, 2) + std::pow(yCoordA - yCoordB, 2));
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
