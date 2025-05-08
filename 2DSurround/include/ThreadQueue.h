#ifndef THREAD_QUEUE_H
#define THREAD_QUEUE_H

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <atomic>

using namespace std;


// 多相机数据同步
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mtx;
    std::condition_variable cond;
    const size_t MAX_SIZE = 500;   // 根据内存调整 6G
    std::atomic<bool>& stop_flag;  // 全局停止标志
public:
    ThreadSafeQueue(std::atomic<bool>& stop) : stop_flag(stop) {}  
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        // 等待条件：队列未满 或 需要停止
        cond.wait(lock, [this] { return queue.size() < MAX_SIZE || stop_flag.load(); });
        if (stop_flag.load()) return; 
        queue.push(item);
        cond.notify_one();
    }
    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        if (queue.empty()) return false;
        item = queue.front();
        queue.pop();
        return true;
    }

    void notify_all() {
        cond.notify_all();  // 唤醒所有等待线程
    }
    void wait_and_pop(T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        cond.wait(lock, [this]{ return !queue.empty(); });
        item = queue.front();
        queue.pop();
    }
    size_t size() {
        std::lock_guard<std::mutex> lock(mtx);
        return queue.size();
    }
};

#endif