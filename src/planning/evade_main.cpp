#include <planning/evade.hpp>
#include <common/getopt.h>
#include <atomic>
#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
    // Define all command-line arguments
    const char* kTeamNumArg = "team-number";

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_int(gopt, 'n', kTeamNumArg, "-1", "Team number of the robot doing the exploration.");

    // If help was requested or the command line is invalid, display the help message and exit
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    // Convert all command-line values into program variables
    int teamNumber = getopt_get_int(gopt, kTeamNumArg);

    // Instantiate the LCM instance and Exploration instance that will run on the two program threads
    lcm::LCM lcmInstance(MULTICAST_URL);

    if(!lcmInstance.good()) return 1;

    Evade evade(teamNumber, &lcmInstance);

    std::atomic<bool> evadeComplete;

    evadeComplete=false;

    // Launch the exploration thread

    std::thread evadeThread([&evade, &evadeComplete]() {
        bool success = evade.evade();
        evadeComplete = true;

        std::cout << "Evade thread complete: " << (success ? "SUCCESS!" : "FAILED!") << '\n';
    });

    // Handle LCM messages until exploration is finished
    while(!evadeComplete)
    {
        lcmInstance.handleTimeout(100);
    }

    // Cleanup the remaining resources
    evadeThread.join();

    std::cout << "Joined explore thread. Exiting\n";

    return 0;
}
