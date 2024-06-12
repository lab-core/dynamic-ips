//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "data/Instance.h"
#include "solver/MasterAlgorithm.h"
#include "solver/LabelingSubProblem.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"

//-----------------------------------------------------------------------------
//  solver class
//  Main algorithms to solve the problem in anytime, fixed epoch or static mode
//-----------------------------------------------------------------------------

class solver {
public:
    PInstance mainInstance_;
    myTools::Timer *simulationTime_;

    solver();
    void createInstance(const std::string& jsonStrDuration, const std::string& jsonStrParam,
                                    const std::string& jsonStrTasks, const std::string& jsonStrVehicles);

    void createInstanceFile(const std::string& strDurFile, const std::string& strParamFile,
                                        const std::string& strTaskFile, const std::string& strVehicleFile);

    virtual ~solver();

    // this function is to solve the epoch instance with CG using ISUD
    std::string solveCG();
};


#endif //SOLVER_H
