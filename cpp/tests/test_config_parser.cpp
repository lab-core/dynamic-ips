//
// Unit tests for ConfigParser::parseArguments: enum-name parsing for
// --main-algo / --sol-mode, optional-argument defaults, and validation.
//
#include "test_framework.h"
#include "utilities/ConfigParser.h"
#include "utilities/Types.h"

#include <memory>
#include <string>
#include <vector>

namespace {

// Run parseArguments on a list of arguments (excluding argv[0], which is added
// here). Returns the parse result; the populated config is returned via out.
bool parse(const std::vector<std::string>& args, PConfig& out) {
    std::vector<std::string> all{"realtime_DARP"};
    all.insert(all.end(), args.begin(), args.end());
    std::vector<char*> argv;
    argv.reserve(all.size());
    for (auto& s : all) argv.push_back(const_cast<char*>(s.c_str()));
    out = std::make_unique<ProgramConfig>();
    return ConfigParser::parseArguments(static_cast<int>(argv.size()), argv.data(), out);
}

// A minimal set of the still-required arguments, with algo/mode as names.
std::vector<std::string> requiredArgs() {
    return {"--inst-folder", "inst", "--main-algo", "B_CG",
            "--sol-mode", "DYNAMIC", "--paramfile", "p", "--scenario", "s"};
}

}  // namespace

TEST(main_algo_and_sol_mode_accept_names) {
    PConfig cfg;
    CHECK(parse(requiredArgs(), cfg));
    CHECK_EQ(cfg->mainAlgo_, static_cast<int>(B_CG));   // 2
    CHECK_EQ(cfg->solMode_, static_cast<int>(DYNAMIC));  // 1
}

TEST(main_algo_and_sol_mode_accept_numbers) {
    auto args = requiredArgs();
    args[3] = "2";  // --main-algo
    args[5] = "1";  // --sol-mode
    PConfig cfg;
    CHECK(parse(args, cfg));
    CHECK_EQ(cfg->mainAlgo_, 2);
    CHECK_EQ(cfg->solMode_, 1);
}

TEST(enum_names_are_case_insensitive) {
    auto args = requiredArgs();
    args[3] = "a_cg";
    args[5] = "anytime";
    PConfig cfg;
    CHECK(parse(args, cfg));
    CHECK_EQ(cfg->mainAlgo_, static_cast<int>(A_CG));    // 6
    CHECK_EQ(cfg->solMode_, static_cast<int>(ANYTIME));  // 2
}

TEST(unknown_main_algo_is_rejected) {
    auto args = requiredArgs();
    args[3] = "NOT_AN_ALGO";
    PConfig cfg;
    CHECK(!parse(args, cfg));
}

TEST(out_of_range_numeric_algo_is_rejected) {
    auto args = requiredArgs();
    args[3] = "99";
    PConfig cfg;
    CHECK(!parse(args, cfg));
}

TEST(optional_arguments_use_defaults) {
    PConfig cfg;
    CHECK(parse(requiredArgs(), cfg));
    CHECK_EQ(cfg->vehicleFolder_, std::string("vehicles"));  // default folder
    CHECK_EQ(cfg->numVehicles_, -1);                         // auto: from file
    CHECK_EQ(cfg->vehicleCapacity_, -1);                     // auto: from file
    CHECK_EQ(cfg->initialState_, 0);                         // default fresh start
}

TEST(explicit_optional_arguments_are_honored) {
    auto args = requiredArgs();
    for (const auto& extra : {std::string("--vehicle-folder"), std::string("fleet"),
                              std::string("--num-vehicles"), std::string("7"),
                              std::string("--vehicle-capacity"), std::string("5"),
                              std::string("--initial-state"), std::string("1")})
        args.push_back(extra);
    PConfig cfg;
    CHECK(parse(args, cfg));
    CHECK_EQ(cfg->vehicleFolder_, std::string("fleet"));
    CHECK_EQ(cfg->numVehicles_, 7);
    CHECK_EQ(cfg->vehicleCapacity_, 5);
    CHECK_EQ(cfg->initialState_, 1);
    // The size-specific vehicle file name is built from --num-vehicles.
    CHECK_EQ(cfg->vehicleFileName_, std::string("vehicles_7_4"));
}

TEST(missing_required_argument_is_rejected) {
    auto args = requiredArgs();
    // Drop "--inst-folder" and its value (the first two entries).
    args.erase(args.begin(), args.begin() + 2);
    PConfig cfg;
    CHECK(!parse(args, cfg));
}

TEST(negative_vehicle_capacity_is_rejected) {
    auto args = requiredArgs();
    args.push_back("--vehicle-capacity");
    args.push_back("-3");
    PConfig cfg;
    CHECK(!parse(args, cfg));
}
