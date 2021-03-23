#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <RVO.h>
#include <vector>

#define FREQUENCY 50

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;
std::vector<geometry_msgs::PoseStamped> poses;

void setupScenario(RVO::RVOSimulator *sim)
{
	std::srand(static_cast<unsigned int>(std::time(NULL)));

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(1.0/FREQUENCY);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.5f, 1.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
	for (size_t i = 0; i < 1; ++i) {
        int j = 0;
		// for (size_t j = 0; j < 1; ++j) {
			sim->addAgent(RVO::Vector2(5.50f + i * 1.00f,  5.50f + j * 1.00f));
			goals.push_back(RVO::Vector2(10.0f, -10.0f));

			sim->addAgent(RVO::Vector2(-5.50f - i * 1.00f,  5.50f + j * 1.00f));
			goals.push_back(RVO::Vector2(10.0f, -10.0f));

			sim->addAgent(RVO::Vector2(5.50f + i * 1.00f, -5.50f - j * 1.00f));
			goals.push_back(RVO::Vector2(-10.0f, 10.0f));

			sim->addAgent(RVO::Vector2(-5.50f - i * 1.00f, -5.50f - j * 1.00f));
			goals.push_back(RVO::Vector2(10.0f, 10.0f));
		// }
	}

	// /*
	//  * Add (polygonal) obstacles, specifying their vertices in counterclockwise order.
	//  */
	// std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

	// obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
	// obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
	// obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
	// obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

	// obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	// obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	// obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	// obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	// obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	// obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	// obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	// obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

	// sim->addObstacle(obstacle1);
	// sim->addObstacle(obstacle2);
	// sim->addObstacle(obstacle3);
	// sim->addObstacle(obstacle4);

	// /* Process the obstacles so that they are accounted for in the simulation. */
	// sim->processObstacles();
}

void updateVisualization(RVO::RVOSimulator *sim)
{
	/* Output the current position of all the agents. */
    poses.resize(sim->getNumAgents());
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        auto temp_pos = sim->getAgentPosition(i);
        poses[i].pose.position.x = temp_pos.x();
        poses[i].pose.position.y = temp_pos.y();
	}
}

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		sim->setAgentPrefVelocity(i, goalVector + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 1.0f * 1.0f) {
			return false;
		}
	}

	return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rvo_simulation");
    ros::NodeHandle nh("~");
    ros::Publisher pedestrian_pose_pub = nh.advertise<nav_msgs::Path>("/pedestrian_positions", 1);
    nav_msgs::Path pedestrian_poses;

    /* Create a new simulator instance. */
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    /* Set up the scenario. */
    setupScenario(sim);

    ros::Rate r(FREQUENCY);
    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        //Do simulation step
        if(!reachedGoal(sim)) {
            updateVisualization(sim);
            setPreferredVelocities(sim);
            sim->doStep();
        }

        else
        {
            ROS_INFO_ONCE("All goals reached");
        }
        
        //Publish new positions of pedestrians
        pedestrian_poses.poses = poses;
        pedestrian_pose_pub.publish(pedestrian_poses);
    }

    delete sim;
    return 0;
}
