//
// Created by Elahe Amiri on 2022-10-23.
//

#include "Tools.h"
using std::mutex;
using std::thread;

//************************************************************************
//                     PThreadsPool implementation
//************************************************************************

namespace Tools {
    int ThreadsPool::maxGlobalThreads_ = 24;
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

    void Task::attach(const PThreadsPool &pThreadsPool) {
        std::unique_lock<mutex> l(mutex_);
        pThreadsPool_ = pThreadsPool;
    }

    void Task::detach() {
        std::unique_lock<mutex> l(mutex_);
        pThreadsPool_->removePtr();
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

    Thread::Thread() : f_(nullptr), needStop_(false) {
        t_ = std::thread([this]() {
            // run an infinite loop while not finished
            std::unique_lock<std::mutex> l(mutex_);
            while (!needStop_) {  // Keep thread alive until told to stop
                if (f_ == nullptr)
                    cWaiting_.wait(l);   // If nothing to do, wait (sleep)
                if (needStop_)              // If need to stop
                    break;
                l.unlock();                 // Else, execute the job
                f_();                       // Execute the job
                l.lock();
                f_ = nullptr;               // once finished, remove f_
                globalThreadsPool.makeAvailable(this);
            }
            delete this;
        });
        t_.detach();                        // Let thread run independently
    }

    void Thread::run(std::function<void(void)> f) {
        std::lock_guard<std::mutex> l(mutex_);
        if (f_ != nullptr)
            myTools::myException::throwError("The thread is already associated to a job.");
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
        for (Thread *pThread: activeThreads)
            pThread->stop();
    }

    void GlobalThreadsPool::run(std::function<void(void)> f) {
        // retrieve an available thread
        std::unique_lock<std::mutex> l(mutex_);
        Thread *pThread;
        if (threadsAvailable_.empty()) {
            pThread = new Thread();                 // Create new if none available
            activeThreads.push_back(pThread);
        } else {
            pThread = threadsAvailable_.front();    // Reuse existing
            threadsAvailable_.pop_front();
        }
        l.unlock();

        // attach the job
        pThread->run(std::move(f));         // Assign job to thread
    }

    void GlobalThreadsPool::makeAvailable(Thread *pThread) {
        std::lock_guard<std::mutex> l(mutex_);
        threadsAvailable_.push_back(pThread);
    }

//************************************************************************
//                     ThreadsPool implementation
//************************************************************************

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

    int ThreadsPool::getNThreadsAvailable(){
        return nThreadsAvailable_;
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
//        setMaxGlobalThreadsToMax();
        return newThreadsPool(maxGlobalThreads_);
    }

    PThreadsPool ThreadsPool::newThreadsPool(int nThreads) {
        return PThreadsPool(new ThreadsPool(nThreads));
    }

    void ThreadsPool::runOneJob(Job job, bool force_detach) {
        newThreadsPool(job.pTask_->nThreads())->run(job, force_detach);
    }

    ThreadsPool::ThreadsPool() : ThreadsPool(maxGlobalThreads_) {}

    ThreadsPool::ThreadsPool(int nThreads) :
            pThreadsPool_(nullptr), nActivePtr_(0) {
        //       setMaxGlobalThreadsToMax();
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
            myTools::myException::throwError("PThreadsPool has been deleted, "
                       "cannot run a new job in this pool.");
        PTask pTask = job.pTask_;
        if (pTask->nThreads() > maxThreads_) {
            std::cerr << "Cannot run on thread pool of " << maxThreads_
                      << " threads a job needing " << pTask->nThreads()
                      << " threads." << std::endl;
            std::cerr << "The job number of threads is thus decreased." << std::endl;
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

    void ThreadsPool::store(const PThreadsPool &pT) {
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

} // Tools namespace
