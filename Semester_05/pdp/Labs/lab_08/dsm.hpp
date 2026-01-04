#pragma once
#include <arpa/inet.h>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <netinet/in.h>
#include <set>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

constexpr int BASE_PORT = 5000;

enum MsgType : uint8_t { WRITE = 1, CAS_REQ = 2, CAS_RESP = 3 };

struct Msg {
  MsgType type;
  int var, val, expected, seq, src, req_id;
  bool ok;
};

class DSM {
  int rank, nprocs;
  std::map<int, int> vars, seqs;
  std::map<int, std::set<int>> subs;
  std::function<void(int, int, int)> on_change;

  std::vector<int> conns;
  int srv_fd = -1;
  std::vector<std::thread> workers;
  std::atomic<bool> alive{true};
  std::mutex mtx;
  std::condition_variable cv;
  std::map<int, std::pair<bool, int>> cas_res;
  std::atomic<int> req_cnt{0};

  int coord(int v) { return *subs[v].begin(); }
  bool subscribed(int v) { return subs.count(v) && subs[v].count(rank); }

  void send_msg(int to, Msg &m) {
    if (to == rank)
      handle(m);
    else
      send(conns[(size_t)to], &m, sizeof(m), 0);
  }

  void handle(Msg &m) {
    std::lock_guard<std::mutex> lk(mtx);
    if (m.type == WRITE) {
      if (m.seq > seqs[m.var]) {
        int old = vars[m.var];
        vars[m.var] = m.val;
        seqs[m.var] = m.seq;
        if (on_change)
          on_change(m.var, old, m.val);
      }
    } else if (m.type == CAS_REQ) {
      Msg r{CAS_RESP, m.var, vars[m.var], 0, 0, m.src, m.req_id, false};
      if (vars[m.var] == m.expected) {
        r.ok = true;
        int s = ++seqs[m.var];
        Msg w{WRITE, m.var, m.val, 0, s, rank, 0, false};
        for (int p : subs[m.var])
          send_msg(p, w);
      }
      send_msg(m.src, r);
    } else if (m.type == CAS_RESP) {
      cas_res[m.req_id] = {m.ok, m.val};
      cv.notify_all();
    }
  }

  void recv_loop(int fd) {
    Msg m;
    while (alive && recv(fd, &m, sizeof(m), 0) == sizeof(m))
      handle(m);
    close(fd);
  }

  void accept_loop() {
    while (alive) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(srv_fd, &fds);
      timeval tv{0, 50000};
      if (select(srv_fd + 1, &fds, 0, 0, &tv) > 0) {
        int c = accept(srv_fd, 0, 0);
        if (c >= 0)
          workers.emplace_back(&DSM::recv_loop, this, c);
      }
    }
  }

public:
  DSM(int r, int n) : rank(r), nprocs(n), conns((size_t)n, -1) {}
  ~DSM() { stop(); }

  void subscribe(int var, std::set<int> who, int init = 0) {
    subs[var] = who;
    if (subscribed(var)) {
      vars[var] = init;
      seqs[var] = 0;
    }
  }

  void set_callback(std::function<void(int, int, int)> f) { on_change = f; }

  void start() {
    srv_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(BASE_PORT + rank);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(srv_fd, (sockaddr *)&addr, sizeof(addr));
    listen(srv_fd, nprocs);
    workers.emplace_back(&DSM::accept_loop, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(50 * nprocs));

    for (int i = 0; i < nprocs; i++) {
      if (i == rank)
        continue;
      int fd = socket(AF_INET, SOCK_STREAM, 0);
      sockaddr_in peer{};
      peer.sin_family = AF_INET;
      peer.sin_port = htons(BASE_PORT + i);
      peer.sin_addr.s_addr = inet_addr("127.0.0.1");
      while (connect(fd, (sockaddr *)&peer, sizeof(peer)) < 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      conns[(size_t)i] = fd;
    }
  }

  void stop() {
    alive = false;
    for (int fd : conns)
      if (fd >= 0)
        close(fd);
    if (srv_fd >= 0) {
      close(srv_fd);
      srv_fd = -1;
    }
    for (auto &t : workers)
      if (t.joinable())
        t.join();
    workers.clear();
  }

  int read(int var) {
    std::lock_guard<std::mutex> lk(mtx);
    return vars[var];
  }

  void write(int var, int val) {
    if (!subscribed(var))
      return;
    std::lock_guard<std::mutex> lk(mtx);
    int s = ++seqs[var];
    Msg m{WRITE, var, val, 0, s, rank, 0, false};
    for (int p : subs[var])
      send_msg(p, m);
  }

  bool cas(int var, int expected, int newval, int *oldval = nullptr) {
    if (!subscribed(var))
      return false;
    if (coord(var) == rank) {
      std::lock_guard<std::mutex> lk(mtx);
      if (oldval)
        *oldval = vars[var];
      if (vars[var] != expected)
        return false;
      int s = ++seqs[var];
      Msg m{WRITE, var, newval, 0, s, rank, 0, false};
      for (int p : subs[var])
        send_msg(p, m);
      return true;
    }
    int id = req_cnt++;
    Msg m{CAS_REQ, var, newval, expected, 0, rank, id, false};
    send_msg(coord(var), m);
    std::unique_lock<std::mutex> lk(mtx);
    cv.wait(lk, [&] { return cas_res.count(id); });
    auto [ok, old] = cas_res[id];
    cas_res.erase(id);
    if (oldval)
      *oldval = old;
    return ok;
  }

  void sync() { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
};

