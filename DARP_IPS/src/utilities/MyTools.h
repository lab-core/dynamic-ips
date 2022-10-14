//
// Created by Ella on 2021-09-07.
//

#ifndef _MYTOOLS_H
#define _MYTOOLS_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <map>
#include <unordered_map>
#include <map>
#include <string>
#include <climits>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include "Eigen/Dense"
#include <set>
#include <queue>
#include <algorithm>
#include <valarray>
#include <thread>  // NOLINT (suppress cpplint error)
#include <mutex>  // NOLINT (suppress cpplint error)
#include <list>
#include <functional>

using std::string;
using std::vector;
using std::chrono::high_resolution_clock;
//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

// useful types
class Instance;
typedef std::shared_ptr<Instance> PInstance;
class Request;
typedef std::shared_ptr<Request> PRequest;
class Vehicle;
typedef std::shared_ptr<Vehicle> PVehicle;
class Graph;
typedef std::shared_ptr<Graph> PGraph;
class Node;
typedef std::shared_ptr<Node> PNode;
class Route;
typedef std::shared_ptr<Route> PRoute;
class ReducedProblem;
typedef std::shared_ptr<ReducedProblem> PReducedProblem;
class ComplementPro;
typedef std::shared_ptr<ComplementPro> PComplementPro;
class ZoomReducedProblem;
typedef std::shared_ptr<ZoomReducedProblem> PZoomReducedProblem;
class Label;
typedef std::shared_ptr<Label> PLabel;
struct Parameters;
typedef std::shared_ptr<Parameters> PParameters;
struct solverOption;
typedef std::shared_ptr<solverOption> PSolverOption;
class GreedyLabel;
typedef std::shared_ptr<GreedyLabel> PGreedyLabel;
class LinkedGreedyLabels;
typedef std::shared_ptr<LinkedGreedyLabels> PLinkedGreedyLabels;
class GreedyModeler;
typedef std::shared_ptr<GreedyModeler> PGreedyModeler;
struct insertPosition;
typedef std::shared_ptr<insertPosition> PInsertPosition;
// extern PTravelTime travelMat;

// SubProblem solution status
enum SubProSolveStart {NOT_RESTRICTED = 0, TIME_RESTRICTED = 1, NUM_PICK_RESTRICTED = 2};
enum LabelingStrategy { PUSHING = 0, PULLING = 1};
enum subproblemAlgorithm { CPLEX = 0, LABEL_SETTING = 1};
enum MainAlgorithm {GREEDY = 0, MIP_CPLEX = 1, CG_CPLEX = 2, CG_ISUD = 3};
enum warmStart {GREEDY_START = 0, PRE_SOLUTION = 1, EMPTY_ROUTES = 2};
enum InitialDual {LAST_CP = 0, PENALTIES = 1};
static const std::vector<std::string> LabelingStrategyName = {
        "PUSHING",
        "PULLING" };

static const std::vector<std::string> subAlgorithmName = {
        "CPLEX        ",
        "LABEL_SETTING" };

static const std::vector<std::string> mainAlgorithmName = {
        "GREEDY",
        "MIP_CPLEX",
        "CG_CPLEX",
        "CG_ISUD"};

static const std::vector<std::string> warmStartName = {
        "GREEDY_START     ",
        "PREVIOUS_SOLUTION",
        "EMPTY_START      "};

static const std::vector<std::string> InitialDualName = {
        "LAST_CP  ",
        "PENALTIES"};
static const std::vector<std::string> SubProSolveStartName = {
        "NOT_RESTRICTED",
        "TIME_RESTRICTED",
        "NUM_PICK_RESTRICTED"};

// Different node types and their names
enum NodeType { SOURCE, SINK, PICKUP, DROPOFF };
/*static const std::vector<std::string> nodeTypeName = {
        "SOURCE ",
        "SINK   ",
        "PICKUP ",
        "DROPOFF"
};*/
static const char *NodeTypeStr[] = {
        "SOURCE ",
        "SINK   ",
        "PICKUP ",
        "DROPOFF"
};

#define MAXReachTime 9999999

static const int DECIMALS = 3;          // precision when printing floats
static const float TimePerMile = 10;   // travel time per mile distance
static const int sentenceSize = 47;


// Definition of useful types
template<class T> using vector2D = std::vector<std::vector<T>>;
template<class T> using vector3D = std::vector<vector2D<T>>;
extern vector2D<float> durationMatrix_;

class PCompare {
public:
    template<typename T>
    bool operator () (std::shared_ptr<T> a, std::shared_ptr<T> b) {
        return (*a) < (*b);
    }
};


namespace Tools {
    // class for defining exception errors

    // class for defining exception errors
    class myException : public std::exception
    {
    public:
        // Constructor
        myException(const char* Msg, int Line)
        {
            std::ostringstream oss;
            oss << "Error line" << Line << " : " << Msg;
            this->msg = oss.str();
        }

        // Destructor
        ~myException() noexcept override = default;

        // Returns a pointer to the (constant) error description
        /* The underlying memory is in possession of the Except object.
         * Callers must not attempt to free the memory */
         const char* what() const noexcept override { return this->msg.c_str();}
    private:
        std::string msg;
    };

    // Throw an exception with the input message
    struct INMsgException: std::exception {
        explicit INMsgException(const char* what): std::exception(), what_(what) {}
        const char* what() const noexcept override {
            return what_;
        }
    private:
        const char* what_;
    };

    void throwException(const char* exceptionMsg);
    void throwException(const std::string& strMsg);
    void throwError(const std::string& strMsg);
    void throwError(const char* exceptionMsg);

    //-----------------------------------------------------------------------------
    //  Functions for calculating the shortest spherical
    //  distance between to points on earth surface
    //-----------------------------------------------------------------------------

    // function to convert degrees to radians
    double toRadians(double degree);

    // function to calculate the distance
    double calcDistance(double lat1, double long1, double lat2, double long2);

    // function to calculate travel time between two coordinate
    double calcTravelTime(double lat1, double long1, double lat2, double long2);

    // functions to create node IDs
    std::string createNodeID(unsigned int requestID, NodeType type);
    std::string createSourceID(int vehicleID, NodeType type);

    // function to compare two val_array
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs);

    // return the object at position i (support negative index)
    template<class T>
    const T &get(const std::vector<T> &v, int i) {
        if (i >= 0) return v[i];
        return v[v.size() + i];
    }


    // Appends the values of v2 vector to at the end of v1 vector
    template <typename T>
    std::vector<T> appendVectors(std::vector<T> & v1, std::vector<T> & V2);

    class Timer {
    private:
        high_resolution_clock::time_point cpuInit_;
        std::chrono::duration<double> cpuSinceStart_;
        std::chrono::duration<double> cpuSinceInit_;

        int coStop_;	        //number of times the timer was stopped
        bool isInit_;
        bool isStarted_;
        bool isStopped_;

        // Constructor and Destructor
    public:
        Timer();
        virtual ~Timer();


        void init();        // function fo initialize the timer
        bool isInit() const;      // function to check initialization status
        void start();       // function to start the timer
        void stop();        // function to stop the timer

        // get the time spent since the initialization of the timer and since the last
        // time it was started
        std::chrono::duration<double> dSinceInit();
        std::chrono::duration<double> dSinceStart();
    };

    // Create a pool of threads that can be used to run functions in parallel.
    // Each pool has a certain number of threads available,
    // but there is also a global limit on the number of threads used.
    class ThreadsPool;
    //-----------------------------------------------------------------------------
    //  PThreadsPool Struct
    //-----------------------------------------------------------------------------
    struct PThreadsPool : public std::shared_ptr<ThreadsPool> {
        template<typename ...Args> PThreadsPool(Args... args):  // NOLINT
                std::shared_ptr<ThreadsPool>(args...) {
            store();
        }
        ~PThreadsPool();
        void store();
    };

    //-----------------------------------------------------------------------------
    //  Task Class
    //-----------------------------------------------------------------------------
    class Task {
        int nThreads_;
        std::function<void(void)> func_;
        PThreadsPool pThreadsPool_ = nullptr;

        std::mutex mutex_;
        std::condition_variable cResume_;

        bool needStop_ = false;
        bool needPause_ = false;
        bool paused_ = false;

        friend class Job;
        friend class ThreadsPool;


        explicit Task(std::function<void(void)> f, int _nThreads) :
                func_(std::move(f)), nThreads_(_nThreads) {}

        // run the task
        void run();

        // attach to the pool
        void attach(const PThreadsPool& pThreadsPool);

        // detach from the pool
        void detach();

        // check if the task is still active for the pool (i.e. attached)
        bool active();

        // ask the task to stop
        void askStop();

        // check if the task has been asked to stop
        bool shouldStop();

        // ask the task to pause
        void askPause();

        // check if the task has been asked to pause
        bool shouldPause();

        // pause the current thread and wait to be resumed by another thread
        void pause();

        // resume the paused thread
        void resume();

        // wait for the task to finish
        void wait();


        int nThreads() const { return nThreads_; }

        void nThreads(int n) { nThreads_ = n; }

    };
    typedef std::shared_ptr<Task> PTask;

    // A wrapper around PTask. Also check if a job is associated to a task, i.e.
    // the class Job has not been initialized with the default constructor
    //-----------------------------------------------------------------------------
    //  Job Class
    //-----------------------------------------------------------------------------
    class Job {
    private:
        PTask pTask_ = nullptr;
        friend class ThreadsPool;

    public:
        Job(): pTask_(nullptr) {}
        explicit Job(std::function<void(void)> f, int nThreads = 1) :
                pTask_(new Task(std::move(f), nThreads)) {}

        // true if a task is attached to the job
        bool defined() const {
            return pTask_ != nullptr;
        }

        // check if any active tsk is attached
        bool active() {
            return pTask_ != nullptr && pTask_->active();
        }

        // true if not active
        bool finish() {
            return !active();
        }

        // Ask to the attached task to stop if any
        void askStop() {
            if (pTask_) pTask_->askStop();
        }

        // check if the attached task has been asked to stop if any
        bool shouldStop() {
            if (pTask_) return pTask_->shouldStop();
            return false;
        }

        // ask to the attached task to pause if any
        void askPause() {
            if (pTask_) pTask_->askPause();
        }

        // check if the attached task has been asked to pause if any
        bool shouldPause() {
            if (pTask_) return pTask_->shouldPause();
            return false;
        }

        // run the task
        void run();

        // pause the current thread and wait to be resumed by another thread
        // if any attached task
        void pause();

        // resume the paused thread if any attached task
        void resume();

        // wait for the task to finish
        void wait();

    };
    //-----------------------------------------------------------------------------
    //  Threads Class
    //-----------------------------------------------------------------------------
    class Thread {
    private:
        std::function<void(void)> f_;
        bool needStop_;
        std::mutex mutex_;
        std::condition_variable cWaiting_;
        std::thread t_;

    public:
        Thread();

        ~Thread() = default;

        void stop();
        void run(std::function<void(void)> f);
    };

    //-----------------------------------------------------------------------------
    //  GlobalThreadsPool Class
    //-----------------------------------------------------------------------------

    class GlobalThreadsPool {
    private:
        std::list<Thread*> threadsAvailable_, activeThreads;
        std::mutex mutex_;
    public:
        GlobalThreadsPool() = default;
        ~GlobalThreadsPool();

        void run(std::function<void(void)> f);

        void makeAvailable(Thread *pThread);
    };

    //-----------------------------------------------------------------------------
    //  ThreadsPool Class
    //-----------------------------------------------------------------------------
    class ThreadsPool {
    private:
        // global maximum number of threads
        static int maxGlobalThreads_;
        // global number of available threads  for the local pool
        static int nGlobalThreadsAvailable_;
        // true if currently trying to set the maxGlobalThreads_
        static bool settingMaxGlobalThreads_;
        static std::mutex mThreadMutex_;
        static std::condition_variable cThreadReleased_;

        int maxThreads_;  // local maximum number of threads
        // local number of available threads for the local pool
        int nThreadsAvailable_;

        // store the shared_pointer to ensure that a copy is always stored and
        // thus we can control when to destroy each pool. It is necessary to keep
        // at least one copy of the shared pointer until each job running
        // in the pool finishes
        PThreadsPool pThreadsPool_;
        // count the number of jobs that has been run and are still not deleted
        int nActivePtr_;
        std::recursive_mutex mutex_;

        friend class PThreadsPool;
        friend class Task;

    public:
        static int getNGlobalThreadsAvailable();

        static int getMaxGlobalThreads();

        // if maximum number of threads decreases,
        // wait for some to be released if wait is true.
        // otherwise decrease of the number of available threads and print a warning.
        // return the new true maxGlobalThreads_
        static int setMaxGlobalThreads(int maxThreads, bool wait = true);
        static int setMaxGlobalThreadsToMax(bool wait = true);

        // method to create a new ThreadsPool
        static PThreadsPool newThreadsPool();
        static PThreadsPool newThreadsPool(int nThreads);

        // create a pool to run only one job
        static void runOneJob(Job job, bool force_detach = false);

        void run(Job job, bool force_detach = false);

        // wait until all threads of the pool are available.
        // returns true if some threads were still running
        bool wait();

        bool wait(int nThreadsToWait);

        bool available() const { return nThreadsAvailable_ > 0; }

        bool busy() const { return nThreadsAvailable_ < maxThreads_; }

        bool idle() const { return nThreadsAvailable_ == maxThreads_; }

    private:
        ThreadsPool();

        explicit ThreadsPool(int nThreads);

        void reserve(int nThreads);

        void release(int nThreads);

        void store(const PThreadsPool& pT);

        void addPtr(PTask pTask);
        void removePtr();

    };

} // Tools namespace


#endif //_MYTOOLS_H
