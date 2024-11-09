/* Author: Ioan Sucan */
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/State.h>
#include <math.h>
#include <cmath>
#include <ompl/base/SpaceInformation.h>

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

            std::vector<ompl::base::State *> robot1;
            std::vector<ompl::base::State *> robot2;
            std::vector<ompl::base::State *> robot3;
            std::vector<ompl::base::State *> robot4;

            std::set<ompl::base::State *> explored;

            void setRobotNodes(std::vector<ompl::base::State *> r1, std::vector<ompl::base::State *> r2, std::vector<ompl::base::State *> r3, std::vector<ompl::base::State *> r4)
            {
                robot1 = r1;
                robot2 = r2;
                robot3 = r3;
                robot4 = r4;

            }

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
            ompl::base::State * getCompositeStates(ompl::base::StateSpacePtr space, ompl::base::SpaceInformationPtr si_);

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
