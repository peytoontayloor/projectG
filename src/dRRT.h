/* Author: Ioan Sucan */
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gdRRT
           @par Short description
           dRRT is a tree-based motion planner that uses the following
           idea: dRRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           J. Kuffner and S.M. LaValle, dRRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/dRRTpubs.html)
        */

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

            void setRobotNodes(std::vector<ompl::base::State *> r1, std::vector<ompl::base::State *> r2, std::vector<ompl::base::State *> r3, std::vector<ompl::base::State *> r4)
            {
                robot1 = r1;
                robot2 = r2;
                robot3 = r3;
                robot4 = r4;

            }

            // TODO: keeping new 'sampler' here, not sure if this is the best practice, might move
            ompl::base::State * getCompositeStates(ompl::base::StateSpacePtr space);

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
