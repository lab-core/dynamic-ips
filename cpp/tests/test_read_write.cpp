//
// Unit / regression tests for the file readers in ReadWrite.
//
//  * countVehicles: fleet size is derived from the file's row count.
//  * readZones: regression guard for the end-of-file over-read that used to
//    insert a spurious zone with an out-of-range center and crash sortZones().
//
#include "test_framework.h"
#include "utilities/ReadWrite.h"
#include "utilities/MyTools.h"   // extern durationMatrix_
#include "utilities/Types.h"
#include "data/Instance.h"
#include "data/Graph.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Write content to a uniquely named file in the temp directory and return its
// path. The caller removes it when done.
std::string writeTempFile(const std::string& tag, const std::string& content) {
    static int counter = 0;
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("darp_test_" + tag + "_" + std::to_string(counter++) + ".txt");
    std::ofstream out(path);
    out << content;
    out.close();
    return path.string();
}

}  // namespace

TEST(count_vehicles_returns_row_count) {
    const std::string content =
        "COLUMNS\n\nvehicle_ID\ncapacity\ndepart_Time\nend_Time\n"
        "depart_ID\nsink_ID\nzone_ID\n\nVEHICLES_INFO\n"
        " 0 4 0 90000    0    0   1\n"
        " 1 4 0 90000    3    3   2\n"
        " 2 4 0 90000    8    8   3\n";
    const std::string path = writeTempFile("vehicles", content);
    CHECK_EQ(ReadWrite::countVehicles(path), 3);
    std::filesystem::remove(path);
}

TEST(count_vehicles_empty_section_is_zero) {
    const std::string content =
        "COLUMNS\n\nvehicle_ID\ncapacity\ndepart_Time\nend_Time\n"
        "depart_ID\nsink_ID\nzone_ID\n\nVEHICLES_INFO\n";
    const std::string path = writeTempFile("vehicles_empty", content);
    CHECK_EQ(ReadWrite::countVehicles(path), 0);
    std::filesystem::remove(path);
}

TEST(read_zones_no_spurious_zone_at_eof) {
    // sortZones() (called at the end of readZones) indexes durationMatrix_ by
    // zone center, so it must cover every center used below.
    durationMatrix_.assign(12, std::vector<float>(12, 0.0f));

    // Trailing newline after the last row is what triggered the old EOF
    // over-read and a bogus zone with center -1.
    const std::string content =
        "COLUMNS\n\nZone_ID\nLocation_ID\n\nZONE_INFO\n"
        "  1    5\n  2    6\n  3    8\n  4   11\n";
    const std::string path = writeTempFile("zones", content);

    std::vector<PVehicle> vehicles;
    PGraph graph = std::make_shared<Graph>();
    PInstance instance = std::make_shared<Instance>(
        "test", 0.0f, 0, 0, 0, vehicles, 0, 12, graph);

    ReadWrite::readZones(path, instance);  // must not crash
    std::filesystem::remove(path);

    CHECK_EQ(instance->nbZones_, 4);
    CHECK_EQ(instance->zones_.size(), static_cast<std::size_t>(4));
    // Exactly the four declared zones, and no bogus 0 / -1 keys.
    CHECK(instance->zones_.count(1) == 1);
    CHECK(instance->zones_.count(4) == 1);
    CHECK(instance->zones_.count(0) == 0);
    CHECK(instance->zones_.count(-1) == 0);
}
