#include <unistd.h>

#include "coppelia_interface.h"
#include "main_simulation.h"

static LIBRARY simLib;

SIM_DLLEXPORT unsigned char simStart(void *, int)
{
    char curDirAndFile[1024];
    getcwd(curDirAndFile, sizeof(curDirAndFile));
    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
    temp += "/libcoppeliaSim.so";

    simLib = loadSimLibrary(temp.c_str());
    if (simLib == NULL)
    {
        printf("Error: could not find or correctly load coppeliaSim.dll. Cannot start the plugin.\n");
        return (0);
    }
    if (getSimProcAddresses(simLib) == 0)
    {
        printf("Error: could not find all required functions in coppeliaSim.dll. Cannot start the plugin.\n");
        unloadSimLibrary(simLib);
        return (0);
    }

    initSimulation();

    return (11);
}

SIM_DLLEXPORT void simEnd()
{
    cleanSimulation();
    
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void *simMessage(int message, int *auxiliaryData, void *customData, int *replyData)
{
    void *retVal = NULL;

    doSimulation();

    return (retVal);
}
