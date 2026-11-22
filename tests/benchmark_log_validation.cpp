#include "benchmark_log.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    int failures = 0;

    void check(bool condition, const std::string& message)
    {
        if (!condition)
        {
            std::cerr << "[FAIL] " << message << '\n';
            ++failures;
        }
    }

    std::vector<std::string> split_csv_line(const std::string& line)
    {
        std::vector<std::string> columns;
        std::stringstream stream(line);
        std::string column;
        while (std::getline(stream, column, ','))
            columns.push_back(column);
        return columns;
    }

    double parse_number(const std::vector<std::string>& columns, std::size_t index)
    {
        if (index >= columns.size()) return std::nan("");
        try
        {
            return std::stod(columns[index]);
        }
        catch (const std::exception&)
        {
            return std::nan("");
        }
    }
}

int main()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "CameraSkinPhysics.BenchmarkLogValidation.csv";

    {
        std::ofstream stale_file(path, std::ios::out | std::ios::trunc);
        stale_file << "stale previous benchmark data\n";
    }

    {
        csp::benchmark_log log(path);
        log.write_frame(1, 10.0, 1.0, 0.25, 0.05, 0.10, 0.10, 0.5, 2.0, 3.0, 4.0,
            200000, 0, 123456, 987654, 4, 0.8f, 0.008f, 0.012f, 0.006f, 0.12f, 0.18f, 0.10f, 0.08f);
    }

    std::ifstream input(path);
    check(static_cast<bool>(input), "benchmark CSV must be created");

    std::string header;
    std::string row;
    std::string extra;
    std::getline(input, header);
    std::getline(input, row);
    std::getline(input, extra);
    check(header.find("cpu_fps") != std::string::npos, "CSV header must include CPU FPS");
    check(header.find("camera_constraint_solve_ms") != std::string::npos,
        "CSV header must include camera constraint solve time");
    check(header.find("camera_edge_solve_ms") != std::string::npos,
        "CSV header must include camera edge solve time");
    check(header.find("camera_area_solve_ms") != std::string::npos,
        "CSV header must include camera area solve time");
    check(header.find("camera_bending_solve_ms") != std::string::npos,
        "CSV header must include camera bending solve time");
    check(header.find("camera_gather_ms") != std::string::npos,
        "CSV header must include camera gather time");
    check(header.find("gpu_total_ms") != std::string::npos, "CSV header must include total GPU time");
    check(header.find("gpu_fps") != std::string::npos, "CSV header must include GPU FPS");
    check(header.find("solver_buffer_bytes") != std::string::npos,
        "CSV header must include selected solver buffer size");
    check(header.find("resident_simulation_buffer_bytes") != std::string::npos,
        "CSV header must include resident simulation buffer size");
    check(header.find("edge_relaxation") != std::string::npos,
        "CSV header must include runtime solver parameters");
    check(header.find("shape_matching_strength") != std::string::npos,
        "CSV header must include cluster shape matching strength");
    check(header.find("stale") == std::string::npos, "CSV must discard previous run data");
    check(extra.empty(), "CSV must contain exactly one data row");

    const std::vector<std::string> columns = split_csv_line(row);
    check(columns.size() == 27, "CSV data row must contain twenty-seven columns");
    check(parse_number(columns, 0) == 1.0, "frame index must be recorded");
    check(std::abs(parse_number(columns, 2) - 100.0) < 1.0e-9,
        "CPU FPS must be derived from CPU frame time");
    check(std::abs(parse_number(columns, 4) - 0.25) < 1.0e-9,
        "camera constraint solve time must be recorded");
    check(std::abs(parse_number(columns, 5) - 0.05) < 1.0e-9,
        "camera edge solve time must be recorded");
    check(std::abs(parse_number(columns, 6) - 0.10) < 1.0e-9,
        "camera area solve time must be recorded");
    check(std::abs(parse_number(columns, 7) - 0.10) < 1.0e-9,
        "camera bending solve time must be recorded");
    check(std::abs(parse_number(columns, 8) - 0.5) < 1.0e-9,
        "camera gather time must be recorded");
    check(std::abs(parse_number(columns, 12) - 10.0) < 1.0e-9,
        "GPU total time must sum the four top-level GPU intervals");
    check(std::abs(parse_number(columns, 13) - 100.0) < 1.0e-9,
        "GPU FPS must be derived from total GPU time");
    check(parse_number(columns, 11) == 200000.0, "particle count must be recorded");
    check(parse_number(columns, 12) == 0.0, "tetrahedron count must be recorded");
    check(parse_number(columns, 13) == 123456.0, "selected solver memory must be recorded");
    check(parse_number(columns, 14) == 987654.0, "resident simulation memory must be recorded");
    check(parse_number(columns, 15) == 4.0, "solver iteration count must be recorded");
    check(std::abs(parse_number(columns, 16) - 0.8) < 1.0e-6,
        "edge relaxation must be recorded");
    check(std::abs(parse_number(columns, 22) - 0.10) < 1.0e-6,
        "contact band must be recorded");
    check(std::abs(parse_number(columns, 23) - 0.08) < 1.0e-6,
        "cluster shape matching strength must be recorded");

    input.close();
    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    check(!remove_error, "temporary benchmark CSV must be removable");

    if (failures != 0)
    {
        std::cerr << failures << " benchmark log validation checks failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Benchmark CSV columns, FPS calculations and memory fields passed.\n";
    return EXIT_SUCCESS;
}
