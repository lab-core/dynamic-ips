//
// Created by Ella on 2021-09-06.
//

#ifndef _INPUTPATHS_H
#define _INPUTPATHS_H

#include <sstream>
#include <string>

//-----------------------------------------------------------------------------
//  Input Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

class InputPaths {
protected:
    std::string instanceName_;
    std::string tripData_;
    std::string instanceData_;
    double timeOUt = 3600;

public:

    // Constructors
    InputPaths();
    InputPaths(std::string datadir, std::string instanceName, double timeOUt = 3600.0);

    // getters
    const std::string &getInstanceName() const;
    const std::string &getTripData() const;
    const std::string &getInstanceData() const;
    double getTimeOUt() const;

    // setters
    void setInstanceName(const std::string &instanceName);
    void setTripData(const std::string &tripData);
    void setInstanceData(const std::string &instanceData);
    void setTimeOUt(double timeOUt);

};


#endif //_INPUTPATHS_H
