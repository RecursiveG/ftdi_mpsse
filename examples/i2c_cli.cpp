#include <arpa/inet.h>
#include <ftdi.h>
#include <histedit.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "mpsse_protocol.h"

#define DIE_IF(cond, fmt, ...)                                                                          \
  do {                                                                                                  \
    if (cond) {                                                                                         \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                                         \
      exit(1);                                                                                          \
    }                                                                                                   \
  } while (0)

using mpsse_protocol::FtdiDevice;
using mpsse_protocol::MpsseI2c;
using mpsse_protocol::Status;

namespace {

// Trim from the start (in place)
inline void ltrim(std::string &s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}

// Trim from the end (in place)
inline void rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
          s.end());
}

inline void trim(std::string &s) {
  ltrim(s);
  rtrim(s);
}

inline bool is_prefix(std::string prefix, std::string s) {
  auto res = std::mismatch(prefix.begin(), prefix.end(), s.begin());
  return res.first == prefix.end();
}

const char *GetPrompt(EditLine *el) { return "i2c>"; }

void PrintHelp() {
  std::printf("Usage:\n");
  std::printf("<dev> is the 7-bits device address\n");
  std::printf("  scan                Scan the bus\n");
  std::printf("  read <dev> <reg>    Read one byte from device's register\n");
}

} // namespace

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev = FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_B);
  DIE_IF(dev == nullptr, "Cannot open dev");
  std::unique_ptr<MpsseI2c> i2c = MpsseI2c::Create(dev.get());
  DIE_IF(i2c == nullptr, "Cannot open i2c");

  History *hist;
  HistEvent hist_ev;
  hist = history_init();
  DIE_IF(hist == nullptr, "history_init fail");
  int errcode = history(hist, &hist_ev, H_SETSIZE, 1000);
  DIE_IF(errcode != 0, "Hist set size fail");
  errcode = history(hist, &hist_ev, H_SETUNIQUE, 1);
  DIE_IF(errcode != 0, "Hist set unique fail");

  EditLine *el;
  el = el_init(argv[0], stdin, stdout, stderr);
  DIE_IF(el == nullptr, "el_init fail");
  errcode = el_set(el, EL_PROMPT, &GetPrompt);
  DIE_IF(errcode != 0, "Set prompt fail");
  errcode = el_set(el, EL_EDITOR, "emacs");
  DIE_IF(errcode != 0, "Set editor mode fail");
  errcode = el_set(el, EL_SIGNAL, 1);
  DIE_IF(errcode != 0, "Set signal fail");
  errcode = el_set(el, EL_HIST, history, hist);
  DIE_IF(errcode != 0, "Set history fail");

  Tokenizer *tok = tok_init(nullptr);
  DIE_IF(tok == nullptr, "tok init fail");

  PrintHelp();

  while (true) {
    tok_reset(tok);
    int count = 0;
    const char *line_c = el_gets(el, &count);
    DIE_IF(count < 0, "el_gets fail");
    std::string line(line_c, count);
    trim(line);
    if (line.empty()) continue;
    if (line == "exit" || line == "quit") break;

    errcode = history(hist, &hist_ev, H_ENTER, line.c_str());
    DIE_IF(errcode < 0, "Failed to log history");
    int line_argc;
    const char **line_argv;
    errcode = tok_str(tok, line.c_str(), &line_argc, &line_argv);
    if (errcode) {
      std::cerr << "Invalid command syntax\n";
      continue;
    }
    DIE_IF(line_argc <= 0, "Parsing internal error");
    std::vector<std::string> str_args;
    for (int i = 0; i < line_argc; i++) {
      str_args.emplace_back(line_argv[i]);
    }

    auto match = [&](std::string prefix, int argcount) -> bool {
      return is_prefix(prefix, line) && line_argc == argcount;
    };

    if (match("help", 1)) {
      PrintHelp();
    } else if (match("scan", 1)) {
      for (int dev = 8; dev < 0x80; dev++) {
        Status st = i2c->Transaction(dev, nullptr, 0, nullptr, 0);
        std::printf("Device addr 0x%02x : %s\n", dev, st.human().c_str());
      }
    } else if (match("read", 3)) {
      std::printf("Not implemented\n");
    } else {
      std::printf("Unknown command\n");
    }
  }

  tok_end(tok);
  el_end(el);
  history_end(hist);

  return 0;
}
