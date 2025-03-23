//
// Created by qzj on 23-5-19.
//

#ifndef SRC_TIC_TOC_H
#define SRC_TIC_TOC_H

#include <chrono>

class TicToc {
public:
    TicToc() {
        tic();
        duration_ms = 0;
        duration_s = 0;
    }

    void tic() {
        start = std::chrono::steady_clock::now();
    }

    double toc() {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        duration_s = elapsed_seconds.count();
        duration_ms = elapsed_seconds.count() * 1000;
		tic();
        return duration_ms;
    }

    double time_used() {
        return duration_s;
    }

    double time_used_ms() {
        return duration_ms;
    }

private:
    std::chrono::steady_clock::time_point start, end;
    double duration_s, duration_ms;
};

#endif //SRC_TIC_TOC_H
