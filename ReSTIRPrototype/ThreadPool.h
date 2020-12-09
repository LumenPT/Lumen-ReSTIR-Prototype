#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

/*
 * https://github.com/progschj/ThreadPool
 *
 * All credits to the original creator of this ThreadPool utility.
 * Modified it a bit though.
 */
class ThreadPool {
public:
	ThreadPool(size_t);

	/*
	 * Get the amount of threads that is currently idle.
	 */
	std::uint16_t numIdleThreads() const;

	/*
	 * Get the amount of threads in total.
	 */
	std::uint16_t numThreads() const;

	/*
	 * Get the amount of threads that is currently performing a task.
	 */
	std::uint16_t numBusyThreads() const;

	/*
	 * Add a task to the queue.
	 */
	void enqueue(std::function<void()> task);

	~ThreadPool();
private:
	//The amount of threads.
	std::size_t size;

	// need to keep track of threads so we can join them
	std::vector<std::thread> workers;
	// the task queue
	std::queue<std::function<void()>> tasks;

	//Count how many threads are idling.
	std::atomic_short idleThreads;

	// synchronization
	std::mutex queue_mutex;
	std::condition_variable condition;
	bool stop;
};

// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads) : size(threads), idleThreads(threads), stop(false)
{
	for (size_t i = 0; i < threads; ++i)
		workers.emplace_back(
			[this]
	{
		for (;;)
		{
			std::function<void()> task;

			{
				std::unique_lock<std::mutex> lock(this->queue_mutex);

				this->condition.wait(lock, [this]
				{
					return this->stop || !this->tasks.empty();
				});

				if (this->stop && this->tasks.empty())
				{
					return;
				}

				task = std::move(this->tasks.front());
				this->tasks.pop();
				--idleThreads;
			}

			task();
			++idleThreads;
		}
	}
	);
}

inline std::uint16_t ThreadPool::numIdleThreads() const
{
	return idleThreads;
}

inline std::uint16_t ThreadPool::numThreads() const
{
	return static_cast<std::int32_t>(size);
}

inline std::uint16_t ThreadPool::numBusyThreads() const
{
	return numThreads() - numIdleThreads();
}

// add new work item to the pool
inline void ThreadPool::enqueue(std::function<void()> task)
{
	std::unique_lock<std::mutex> lock(queue_mutex);

	// don't allow enqueueing after stopping the pool
	if (stop)
		throw std::runtime_error("enqueue on stopped ThreadPool");

	tasks.emplace(task);

	condition.notify_one();
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
	{
		std::unique_lock<std::mutex> lock(queue_mutex);
		stop = true;
	}
	condition.notify_all();
	for (auto& worker : workers)
		worker.join();
}
