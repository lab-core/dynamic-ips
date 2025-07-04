//
// Created by Jérémy Omer on 19/11/2013.
// Copyright (c) 2013 Jérémy Omer. All rights reserved.
//

#ifndef _TOOLS_H
#define _TOOLS_H

#include <thread>  // NOLINT (suppress cpplint error)
#include <mutex>  // NOLINT (suppress cpplint error)
#include <list>
#include <functional>
#include <iostream>
#include "MyTools.h"
#include <condition_variable>
#include <iomanip>

namespace Tools{
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

        // Ask to stop the attached task if any
        void askStop() {
            if (pTask_) pTask_->askStop();
        }

        // check if the attached task has been asked to stop if any
        bool shouldStop() {
            if (pTask_) return pTask_->shouldStop();
            return false;
        }

        // Ask to pause the attached task if any
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
        std::list<Thread*> threadsAvailable_;           // Reusable threads
        std::list<Thread*> activeThreads;               // All created threads
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
        // global number of available threads for the local pool
        static int nGlobalThreadsAvailable_;
        // true if currently trying to set the maxGlobalThreads_
        static bool settingMaxGlobalThreads_;
        static std::mutex mThreadMutex_;
        static std::condition_variable cThreadReleased_;

        int maxThreads_;  // local maximum number of threads
        // local number of available threads for the local pool
        int nThreadsAvailable_;

        // store the shared_pointer to ensure that a copy is always stored, and
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
        int getNThreadsAvailable();

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


    // Instantiate an object of this class to write directly in the attribute log
// file.
// The class can be initialized with an arbitrary width if all the outputs must
// have the same minimum width. Precision can be set to force a maximum width
// as far as floating numbers are concerned.
//
    class LogOutput
    {
    private:
        std::ostream* pLogStream_;
        int width_;
        int precision_;
        std::string logName_="";

    public:
        LogOutput(std::string logName, bool append = false):width_(0), precision_(5), logName_(logName) {
            if (logName.empty()) {
                pLogStream_ = &(std::cout);
            }
            else if(append) {
                pLogStream_ = new std::ofstream(logName.c_str(), std::fstream::app);
            }
            else {
                pLogStream_ = new std::ofstream(logName.c_str(), std::fstream::out);
            }
        }
        LogOutput(std::string logName, int width, bool append = false):width_(width), precision_(5), logName_(logName) {
            if(append) {
                pLogStream_ = new std::ofstream(logName.c_str(), std::fstream::app);
            }
            else
                pLogStream_ = new std::ofstream(logName.c_str(), std::fstream::out);
        }

        ~LogOutput() {
            if (!logName_.empty() && pLogStream_)
                delete pLogStream_;
            pLogStream_ = NULL;
        }

        void close() {
            std::ofstream* pStream = dynamic_cast<std::ofstream*>(pLogStream_);
            if (pStream) {
                if (pStream->is_open()) pStream->close();
            }
            else pLogStream_ = NULL;
        }

        // switch from un-formatted to formatted inputs and reversely
        //
        void switchToFormatted(int width) {
            width_ = width;
        }
        void switchToUnformatted() {
            width_=0;
        }

        // set flags in the log stream
        //
        void setFlags(std::ios_base::fmtflags flag) {pLogStream_->flags(flag);}

        // modify the precision used to write in the stream
        //
        void setPrecision(int precision) {precision_=precision;}

        // modify the width of the fields
        //
        void setWidth(int width) {width_ = width;}

        void endl() {(*pLogStream_) << std::endl;}

        // redefine the output function
        //
        template<typename T>
        LogOutput& operator<<(const T& output)
        {
            pLogStream_->width(width_);
            pLogStream_->unsetf ( std::ios::floatfield );
            pLogStream_->precision(precision_);
            (*pLogStream_) << std::left << std::setprecision(5) << output;

            return *this;
        }

        // write in the command window as well as in the log file
        //
        template<typename T>
        void print(const T& output) {
            (*pLogStream_) << output;
            std::cout << output;
        }

        LogOutput& operator<<(std::ostream& (*func)(std::ostream&)) {

            func(*pLogStream_);
            return *this;
        }
    };

// Can be used to create an output stream that writes with a
// constant width in all future calls of << operator
//
    class FormattedOutput
    {
    private:
        int width;
        std::ostream& stream_obj;

    public:
        FormattedOutput(std::ostream& obj, int w): width(w), stream_obj(obj) {}

        template<typename T>
        FormattedOutput& operator<<(const T& output)
        {
            stream_obj.width(width);
            stream_obj << output;

            return *this;
        }

        FormattedOutput& operator<<(std::ostream& (*func)(std::ostream&))
        {
            func(stream_obj);
            return *this;
        }
    };

} // Tools namespace


#endif //_TOOLS_H
