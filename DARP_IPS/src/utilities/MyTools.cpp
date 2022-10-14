//
// Created by Ella on 2021-09-07.
//
#define _USE_MATH_DEFINES

#include "MyTools.h"
#include <cmath>
#include <chrono>

using std::mutex;
using std::thread;

//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

namespace Tools {

    // Throw an exception with the input message
    void throwException(const char *exceptionMsg) {
        try {
            throw INMsgException(exceptionMsg);
        }
        catch (std::exception& e) {
            printf("Exception caught: %s\n", e.what());
            throw e;
        }
    }

    void throwException(const std::string &strMsg) {
        throwException(strMsg.c_str());
    }


    void throwError(const std::string &strMsg) {
        try {
            throw strMsg;
        }
        catch (std::string& Msg) {
            printf("Error caught: %s\n", Msg.c_str());
            throw Msg;
        }
    }

    void throwError(const char *exceptionMsg) {
        throwError(std::string(exceptionMsg));
    }

    //-----------------------------------------------------------------------------
    //  Functions for calculating the shortest distance between to points on earth
    //-----------------------------------------------------------------------------

    // function to convert degrees to radians
    double toRadians(double degree) {
        double oneDeg = (M_PI)/180;
        return (oneDeg * degree);
    }

    // function to calculate the distance
    double calcDistance(double lat1, double long1, double lat2, double long2) {

        // convert the latitudes and longitudes from degree to radians
        lat1 = toRadians(lat1);
        long1 = toRadians(long1);
        lat2 = toRadians(lat2);
        long2 = toRadians(long2);

        // Haversine Formula
        double dLong = long2 - long1;
        double dLat = lat2 - lat1;

        double dist = pow(sin(dLat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dLong/2),2);
        dist = 2 * asin(sqrt(dist));

        // Radius of the Earth is 6371 km and 3956 miles
        double Radius = 3956;

        return dist * Radius;
    }
    // function to calculate travel time
    double calcTravelTime(double lat1, double long1, double lat2, double long2) {
        double dist = calcDistance(lat1, long1, lat2, long2);
        return dist * TimePerMile;
    }

    // functions to create node IDs
    std::string createNodeID(unsigned int requestID, NodeType type) {
        std::string ID;
        switch(type) {
            case SOURCE :
                ID = "SO_" + std::to_string(requestID);
                break;
            case SINK :
                ID = "SI_" + std::to_string(requestID);
                break;
            case PICKUP :
                ID = "PI_" + std::to_string(requestID);
                break;
            case DROPOFF :
                ID = "DR_" + std::to_string(requestID);
                break;
        }
        return ID;
    }

    std::string createSourceID(int vehicleID, NodeType type) {
        std::string ID;
        if (type == SOURCE)
            ID = "SO_" + std::to_string(vehicleID);
        else if (type == SINK)
            ID = "SI_" + std::to_string(vehicleID);
        return ID;
    }

    // function to compare two val_array
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs) {
        std::valarray<bool> result = rhs <= lhs;
        return std::all_of(begin(result), end(result), [](bool v) { return v; });
    }

    // Appends the values of v2 vector to at the end of v1 vector
    template<typename T>
    vector<T> appendVectors(vector<T> &v1, vector<T> &v2) {
        std::vector<T> ANS;
        for (int i = 0; i < v1.size(); ++i) ANS.push_back(v1[i]);
        for (int i = 0; i < v2.size(); ++i) ANS.push_back(v2[i]);
        return ANS;
    }

    //************************************************************************
    //                     FUNCTIONS FOR TIMER CLASS
    //************************************************************************

    Timer::Timer() : isInit_(false), isStarted_(false), isStopped_(false) {
        this->init();
    }
    Timer::~Timer() = default;

    // function fo initialize the timer and set it in stop mode
    void Timer::init() {
        coStop_ = 0;

        cpuSinceInit_ = high_resolution_clock::duration::zero();
        cpuSinceStart_= high_resolution_clock::duration::zero();

        isInit_ = true;
        isStarted_ = false;
        isStopped_ = true;
    }

    // function to check initialization status
    bool Timer::isInit() const {
        return isInit_;
    }

    // function to start the timer
    void Timer::start() {
        if (isStarted_)
            throwError("Trying to start an already started timer!");
        cpuInit_ = high_resolution_clock::now();
        cpuSinceStart_= high_resolution_clock::duration::zero();
        isStarted_ = true;
        isStopped_ = false;
    } // end start

    // function to stop the timer
    void Timer::stop() {
        if (isStopped_)
            throwError("Trying to stop an already stopped timer!");
        high_resolution_clock::time_point cpuNow;
        cpuNow = high_resolution_clock::now();

        cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);
        cpuSinceInit_ = cpuSinceInit_ + cpuSinceStart_;
        isStarted_ = false;
        isStopped_ = true;
        coStop_ ++;
    } // end stop

    // get the time spent since the initialization of the timer and since the last
    // time it was started
    std::chrono::duration<double> Timer::dSinceInit() {
        std::chrono::duration<double> cpuCurrent;
        if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();

            std::chrono::duration<double> cpuTmp;
            cpuTmp = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);


            cpuCurrent = cpuTmp + cpuSinceInit_;
            return cpuCurrent;
        }
        else if (isStopped_) {
            return cpuSinceInit_;
        }
        else
            throwError("Trying to get the value of an uninitialized timer!");
        return high_resolution_clock::duration::zero();
    } // end dSinceInit

    // Get the time spent since the last start of the timer without stopping it
    std::chrono::duration<double> Timer::dSinceStart() {
        if (!isStarted_ && !isStopped_)
            throwError("Trying to get the value of an uninitialized timer!");
        else if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();
            cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);

        }
        return cpuSinceStart_;
    } //end dSinceStart

    //************************************************************************
    //                     PThreadsPool implementation
    //************************************************************************
    PThreadsPool::~PThreadsPool() {
        if (get())
            get()->removePtr();
    }

    void PThreadsPool::store() {
        if (get())
            get()->store(*this);
    }

    //************************************************************************
    //                     Task implementation
    //************************************************************************
    void Task::run() {
        try {
            func_();
        } catch (const std::exception &e) {
            std::cout << "Task::run() caught an exception=: "
                      << e.what() << std::endl;
        }
    }

    void Task::attach(const PThreadsPool& pThreadsPool) {
        std::unique_lock<mutex> l(mutex_);
        pThreadsPool_ = pThreadsPool;
    }

    void Task::detach() {
        std::unique_lock<mutex> l(mutex_);
        pThreadsPool_ = nullptr;
        cResume_.notify_all();
    }

    bool Task::active() {
        std::unique_lock<mutex> l(mutex_);
        return pThreadsPool_ != nullptr;
    }

    void Task::askStop() {
        std::unique_lock<mutex> l(mutex_);
        needStop_ = true;
        needPause_ = false;
        if (paused_) {
            // reserve again the threads
            pThreadsPool_->reserve(nThreads_);
            // and notify the paused thread
            paused_ = false;
            cResume_.notify_all();
        }
    }

    bool Task::shouldStop() {
        std::unique_lock<mutex> l(mutex_);
        return needStop_;
    }

    void Task::askPause() {
        std::unique_lock<mutex> l(mutex_);
        needPause_ = true;
    }

    bool Task::shouldPause() {
        std::unique_lock<mutex> l(mutex_);
        return needPause_;
    }

    void Task::pause() {
        std::unique_lock<mutex> l(mutex_);
        if (pThreadsPool_ == nullptr) return;
        // release the threads
        pThreadsPool_->release(nThreads_);
        // then wait
        needPause_ = false;
        paused_ = true;
        cResume_.wait(l);
    }

    void Task::resume() {
        std::unique_lock<mutex> l(mutex_);
        if (pThreadsPool_ == nullptr || !paused_) return;
        // reserve again the threads
        pThreadsPool_->reserve(nThreads_);
        // and notify the paused thread
        paused_ = false;
        cResume_.notify_all();
    }

    void Task::wait() {
        std::unique_lock<mutex> l(mutex_);
        while (pThreadsPool_ != nullptr)
            cResume_.wait(l);
    }

    //************************************************************************
    //                     Job implementation
    //************************************************************************
    void Job::run() {
        if (pTask_ != nullptr)
            pTask_->run();
    }

    void Job::pause() {
        if (pTask_ != nullptr)
            pTask_->pause();
    }

    void Job::resume() {
        if (pTask_ != nullptr)
            pTask_->resume();
    }

    void Job::wait() {
        if (pTask_ != nullptr)
            pTask_->wait();
    }

    GlobalThreadsPool globalThreadsPool;
    //************************************************************************
    //                     FUNCTIONS FOR Threads CLASS
    //************************************************************************

    Thread::Thread(): f_(nullptr), needStop_(false) {
        t_ = std::thread([this]() {
            // run an infinite loop while not finished
            std::unique_lock<std::mutex> l(mutex_);
            while (!needStop_) {
                // if nothing to do, wait
                if (f_ == nullptr)
                    cWaiting_.wait(l);
                // if need to stop
                if (needStop_)
                    break;
                // else run
                l.unlock();
                f_();
                // once finished, remove f_
                l.lock();
                f_ = nullptr;
                globalThreadsPool.makeAvailable(this);
            }
            delete this;
        });
        t_.detach();
    }

    void Thread::run(std::function<void(void)> f) {
        std::lock_guard<std::mutex> l(mutex_);
        if (f_ != nullptr)
            Tools::throwError("The thread is already associated to a job.");
        f_ = std::move(f);
        cWaiting_.notify_one();
    }

    void Thread::stop() {
        std::lock_guard<std::mutex> l(mutex_);
        needStop_ = true;
        cWaiting_.notify_one();
    }
    //************************************************************************
    //                     GlobalThreadsPool implementation
    //************************************************************************
    GlobalThreadsPool::~GlobalThreadsPool() {
        for (Thread *pThread : activeThreads)
            pThread->stop();
    }

    void GlobalThreadsPool::run(std::function<void(void)> f) {
        // retrieve an available thread
        std::unique_lock<std::mutex> l(mutex_);
        Thread *pThread;
        if (threadsAvailable_.empty()) {
            pThread = new Thread();
            activeThreads.push_back(pThread);
        } else {
            pThread = threadsAvailable_.front();
            threadsAvailable_.pop_front();
        }
        l.unlock();

        // attach the job
        pThread->run(std::move(f));
    }

    void GlobalThreadsPool::makeAvailable(Thread *pThread) {
        std::lock_guard<std::mutex> l(mutex_);
        threadsAvailable_.push_back(pThread);
    }

    //************************************************************************
    //                     ThreadsPool implementation
    //************************************************************************

    int ThreadsPool::maxGlobalThreads_ = 1;
    int ThreadsPool::nGlobalThreadsAvailable_ = ThreadsPool::maxGlobalThreads_;
    bool ThreadsPool::settingMaxGlobalThreads_ = false;
    std::mutex ThreadsPool::mThreadMutex_;
    std::condition_variable ThreadsPool::cThreadReleased_;

    int ThreadsPool::getNGlobalThreadsAvailable() {
        return nGlobalThreadsAvailable_;
    }

    int ThreadsPool::getMaxGlobalThreads() {
        return maxGlobalThreads_;
    }

    int ThreadsPool::setMaxGlobalThreadsToMax(bool wait) {
        return ThreadsPool::setMaxGlobalThreads(
                thread::hardware_concurrency(), wait);
    }

    int ThreadsPool::setMaxGlobalThreads(int maxThreads, bool wait) {
        std::unique_lock<mutex> l(mThreadMutex_);

        // set to true the flag settingMaxGlobalThreads_
        // if already setting the max number of threads, print a warning and return
        if (settingMaxGlobalThreads_) {
            std::cerr
                    << "WARNING: another thread is already trying to "
                       "change the max number of available threads"
                    << std::endl;
            return maxGlobalThreads_;
        }
        settingMaxGlobalThreads_ = true;

        // check max number of cores on the system
        int sysThreads = thread::hardware_concurrency();
        if (maxThreads > sysThreads) {
            std::cerr << "WARNING: the system has " << sysThreads
                      << " cores, and we are trying to set the maximum threads to "
                      << maxThreads
                      << ": the maximum is automatically blocked at " << sysThreads
                      << " threads." << std::endl;
            maxThreads = sysThreads;
        }

        // try to update the max
        int diff = maxThreads - maxGlobalThreads_;
        // ensure that the maximum can be decreased
        while (nGlobalThreadsAvailable_ + diff < 0) {
            // wait to be able to decrease totally the number of global threads
            if (wait) {
                cThreadReleased_.wait(l);  // wait until notification
            } else {  // just decrease what is currently available
                diff = -nGlobalThreadsAvailable_;
                std::cerr
                        << "WARNING: the maximum global number of threads "
                           "cannot be decreased to "
                        << maxThreads
                        << " as " << (maxGlobalThreads_ - nGlobalThreadsAvailable_)
                        << " of them are currently used."
                        << "The max has been instead set to " << (maxGlobalThreads_ + diff)
                        << "." << std::endl;
            }
        }

        // update the number of threads
        maxGlobalThreads_ += diff;
        nGlobalThreadsAvailable_ += diff;
        settingMaxGlobalThreads_ = false;

        return maxGlobalThreads_;
    }

    PThreadsPool ThreadsPool::newThreadsPool() {
        setMaxGlobalThreadsToMax();
        return newThreadsPool(maxGlobalThreads_);
    }

    PThreadsPool ThreadsPool::newThreadsPool(int nThreads) {
        return PThreadsPool(new ThreadsPool(nThreads));
    }

    void ThreadsPool::runOneJob(Job job, bool force_detach) {
        newThreadsPool(job.pTask_->nThreads())->run(job, force_detach);
    }

    ThreadsPool::ThreadsPool() : ThreadsPool(maxGlobalThreads_) {}

    ThreadsPool::ThreadsPool(int nThreads):
            pThreadsPool_(nullptr), nActivePtr_(0) {
        if (nThreads <= maxGlobalThreads_) {
            maxThreads_ = nThreads;
        } else {
            maxThreads_ = maxGlobalThreads_;
            std::cerr << "WARNING: the local pool cannot use " << nThreads
                      << " threads, instead it uses the maximum allowed: "
                      << maxGlobalThreads_ << std::endl;
        }
        nThreadsAvailable_ = maxThreads_;
    }

    void ThreadsPool::run(Job job, bool force_detach) {
        if (pThreadsPool_ == nullptr)
            throwError("PThreadsPool has been deleted, "
                       "cannot run a new job in this pool.");
        PTask pTask = job.pTask_;
        if (pTask->nThreads() > maxThreads_) {
            std::cerr << "Cannot run on thread pool of " << maxThreads_
                      << " threads a job needing " << pTask->nThreads()
                      << " threads." << std::endl;
            std:: cerr << "The job number of threads is thus decreased." << std::endl;
            pTask->nThreads(maxThreads_);
        }
        if (pTask->pThreadsPool_ != nullptr) {
            pTask->resume();  // if already attached, resume the job
        } else if (maxGlobalThreads_ <= 1 && !force_detach) {  // no concurrency
            pTask->run();
        } else {
            reserve(pTask->nThreads());  // reserve a thread
            addPtr(pTask);  // attach the task to the pool
            globalThreadsPool.run([this, pTask]() {
                pTask->run();  // run the function
                release(pTask->nThreads());  // release the thread
                pTask->detach();  // detach from the pool
            });
        }
    }


    // wait until all threads of the pool are available.
    // returns true if some threads were still running
    bool ThreadsPool::wait() {
        return wait(maxThreads_);
    }

    bool ThreadsPool::wait(int nThreadsToWait) {
        std::unique_lock<mutex> l(mThreadMutex_);  // create a lock on the mutex
        // all threads of the local pool have ended
        if (nThreadsAvailable_ == maxThreads_)
            return false;
        // wait for all threads to finish
        int threadsAvailableTarget =
                std::min(maxThreads_, nThreadsAvailable_ + nThreadsToWait);
        cThreadReleased_.wait(l, [&]() {
            return !settingMaxGlobalThreads_
                   && nThreadsAvailable_ >= threadsAvailableTarget;
        });
        return true;
    }

    void ThreadsPool::store(const PThreadsPool& pT) {
        std::lock_guard<std::recursive_mutex> l(mutex_);
        nActivePtr_++;
        pThreadsPool_ = pT;
    }

    void ThreadsPool::removePtr() {
        std::lock_guard<std::recursive_mutex> l(mutex_);
        nActivePtr_--;
        // only delete the pointer if there is no active pointer anymore
        if (nActivePtr_ == 0)
            pThreadsPool_ = nullptr;
    }

    void ThreadsPool::addPtr(PTask pTask) {
        std::lock_guard<std::recursive_mutex> l(mutex_);
        nActivePtr_++;
        pTask->attach(pThreadsPool_);  // attach to pool
    }

    void ThreadsPool::reserve(int nThreads) {
        std::unique_lock<mutex> l(mThreadMutex_);
        // reserve ont thread if available, otherwise wait for one
        bool reserved = false;
        while (!reserved) {
            // wait until one thread is available and settingMaxGlobalThreads_ is false
            cThreadReleased_.wait(l, [this]() {
                return nGlobalThreadsAvailable_ > 0 && nThreadsAvailable_ > 0
                       && !settingMaxGlobalThreads_;
            });
            // reserved one thread
            int minThreads = std::min(
                    nThreads, std::min(nGlobalThreadsAvailable_, nThreadsAvailable_));
            nGlobalThreadsAvailable_ -= minThreads;  // reserve global threads
            nThreadsAvailable_ -= minThreads;  // reserve local threads
            nThreads -= minThreads;
            if (nThreads == 0)
                reserved = true;
        }
    }

    void ThreadsPool::release(int nThreads) {
        std::lock_guard<mutex> l(mThreadMutex_);
        // release threads
        nGlobalThreadsAvailable_ += nThreads;
        nThreadsAvailable_ += nThreads;
        cThreadReleased_.notify_all();  // notify any thread that were waiting
    }
} // end namespace



