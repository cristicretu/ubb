#include "service.h"

#include <algorithm>

std::vector<Task> Service::getAllTasks() const {
  std::vector<Task> tasks = repo.getTasks();

  std::sort(tasks.begin(), tasks.end(), [](const Task &a, const Task &b) {
    return a.getPriority() == b.getPriority()
               ? a.getDuration() < b.getDuration()
               : a.getPriority() < b.getPriority();
  });

  return tasks;
}
std::pair<int, std::vector<Task>> Service::getDurationAndTaskByPriority(
    int priority) const {
  std::vector<Task> tasks = repo.getTasks();
  std::vector<Task> ans;
  int total = 0;

  for (auto t : tasks) {
    if (t.getPriority() == priority) {
      ans.push_back(t);
      total += t.getDuration();
    }
  }

  return {total, ans};
}
