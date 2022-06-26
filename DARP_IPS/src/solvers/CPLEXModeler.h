//
// Created by Ella on 2021-10-22.
//

#ifndef _CPLEXMODELER_H
#define _CPLEXMODELER_H

#include "data/Route.h"

//-----------------------------------------------------------------------------
//  Tools to create a cplex program
//-----------------------------------------------------------------------------

typedef IloArray<IloNumVarArray> IloNumVar2D;		// 2-dim array of variables
typedef IloArray<IloNumVar2D> IloNumVar3D;          // 3-dim array of variables
typedef IloArray<IloNumArray> IloNum2D;		        // 2-dim array of variables
typedef IloArray<IloRangeArray> IloRange2D;         // 2-dim range array
typedef IloIterator<IloNumVar> IloNumVarIterator;


// function to create IloNumArray with identical elements
static void createIloNumArray (IloNumArray& numArray, unsigned int size, int elementValue) {
    numArray.clear();
    for (int i = 0; i < size; ++i) {
        numArray.add(elementValue);
    }
}

// function to create pattern from routes
static void createPattern (IloNumArray& pattern, PRoute & route, std::unordered_map<unsigned int, int>& requestToOrder) {
    for (auto requestID : route->routeRequests_) {
        pattern[requestToOrder[requestID]] = 1;
    }
}


#endif //_CPLEXMODELER_H