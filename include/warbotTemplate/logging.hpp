#pragma once
#include "main.h"
#include <functional>
#include <string>
#include <vector>
#include <cstdio>

namespace warbots {

// Returns a fresh path "/usd/<testName>_<n>.csv" for this run. <n> comes from a persistent
// counter file "/usd/<testName>_idx.txt" (plain ASCII int), which is read, incremented, and
// rewritten each call. A missing/corrupt counter file (e.g. first run ever, or no SD card) is
// treated as 0, so numbering starts at 1.
// Note: manually deleting the counter file while old .csv logs remain on the card restarts
// numbering at 1 and can overwrite "<testName>_1.csv" - only happens under manual SD tampering.
inline std::string nextLogPath(const std::string& testName) {
    std::string counterPath = "/usd/" + testName + "_idx.txt";
    int n = 0;
    if (FILE* in = fopen(counterPath.c_str(), "r")) {
        if (fscanf(in, "%d", &n) != 1) n = 0;
        fclose(in);
    }
    n++;
    if (FILE* out = fopen(counterPath.c_str(), "w")) {
        fprintf(out, "%d", n);
        fclose(out);
    }
    return "/usd/" + testName + "_" + std::to_string(n) + ".csv";
}

// One named, sampled CSV column: `name` becomes the header text, `sample` is invoked once per
// row from the logger's background task.
struct LogColumn {
    std::string name;
    std::function<double()> sample;
};

// Samples a set of named columns to a uniquely-named CSV file on a background task, replacing
// the hand-rolled fopen/pros::Task/fprintf/fclose block that used to be duplicated in every
// tuning auton. Declare it after any stack locals its column lambdas capture by reference
// (e.g. a leg counter) - C++ destroys it (stopping the task) before those locals go out of
// scope, the same lifetime guarantee the old manual teardown gave by hand.
class CsvLogger {
public:
    CsvLogger(const std::string& testName, std::vector<LogColumn> cols)
        : testName(testName), columns(std::move(cols)) {}

    CsvLogger(const CsvLogger&) = delete;
    CsvLogger& operator=(const CsvLogger&) = delete;

    void start(int intervalMs = 10) {
        this->intervalMs = intervalMs;
        path = nextLogPath(testName);
        file = fopen(path.c_str(), "w");
        if (file != nullptr) {
            for (size_t i = 0; i < columns.size(); i++) {
                fprintf(file, "%s%s", columns[i].name.c_str(), (i + 1 < columns.size()) ? "," : "\n");
            }
        }
        running = true;
        task.emplace([this]() {
            while (running) {
                writeRow();
                pros::delay(this->intervalMs);
            }
        });
    }

    void stop() {
        if (!running) return;
        running = false;
        if (task.has_value()) {
            task->join();
            task.reset();
        }
        if (file != nullptr) {
            fclose(file);
            file = nullptr;
        }
    }

    ~CsvLogger() { stop(); }

    const std::string& getPath() const { return path; }

private:
    void writeRow() {
        if (file == nullptr) return;
        for (size_t i = 0; i < columns.size(); i++) {
            fprintf(file, "%.3f%s", columns[i].sample(), (i + 1 < columns.size()) ? "," : "\n");
        }
    }

    std::string testName;
    std::vector<LogColumn> columns;
    std::string path;
    FILE* file = nullptr;
    std::optional<pros::Task> task;
    volatile bool running = false;
    int intervalMs = 10;
};

}  // namespace warbots
