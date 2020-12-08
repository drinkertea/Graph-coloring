#pragma once
#include "common.h"

struct TimeoutThread
{
    std::atomic_bool stopped = false;

    TimeoutThread(std::chrono::milliseconds wait_for)
        : m_thread([this, wait_for]() {
        std::this_thread::sleep_for(wait_for);
        stopped = true;
            })
    {
    }

            ~TimeoutThread()
            {
                if (!stopped)
                    TerminateThread(m_thread.native_handle(), 0);
                m_thread.join();
            }

private:
    std::thread m_thread;
};

struct Timer
{
    Timer()
    {
        Reset();
    }

    void Reset()
    {
        start = std::chrono::system_clock::now();
    }

    uint64_t Stop()
    {
        auto int_end = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(int_end - start).count();
    }

private:
    std::chrono::system_clock::time_point start;
};

struct AvgTimer
{
    AvgTimer()
    {
    }

    void Reset()
    {
        timer.Reset();
    }

    void Stop()
    {
        auto elapsed = timer.Stop();
        if (cnt++ == 0)
        {
            avg = double(elapsed);
        }
        else
        {
            avg += (1.0 / (double(cnt))) * (double(elapsed) - avg);
        }
    }

    double GetValue() const
    {
        return avg;
    }

    AvgTimer& operator+=(const AvgTimer& rhs)
    {
        uint32_t sum_cnt = cnt + rhs.cnt;
        avg = avg * (double(cnt) / double(sum_cnt)) + rhs.avg * (double(rhs.cnt) / double(sum_cnt));
        return *this;
    }

private:
    Timer    timer{};
    double   avg = 0;
    uint32_t cnt = 0;
};

struct TimerGuard
{
    TimerGuard(AvgTimer& t)
        : timer(t)
    {
        timer.Reset();
    }

    ~TimerGuard()
    {
        timer.Stop();
    }

private:
    AvgTimer& timer;
};
