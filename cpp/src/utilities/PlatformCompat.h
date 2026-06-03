//
// Small cross-platform filesystem helpers.
//
// These wrap std::filesystem so the rest of the code does not depend on POSIX
// headers (<sys/stat.h>, mkdir, S_ISDIR), keeping it portable across Linux,
// macOS, and Windows.
//
#pragma once

#include <filesystem>
#include <string>

namespace platform {

/// @brief Test whether a path exists and is a directory.
///
/// @param path Path to test.
/// @return true if the path exists and is a directory.
inline bool pathIsDirectory(const std::string& path) {
    std::error_code ec;
    return std::filesystem::is_directory(path, ec);
}

/// @brief Create a single directory (its parent must already exist).
///
/// Mirrors the previous mkdir() semantics: it reports success only when the
/// directory is newly created, so callers can treat "already exists" as a
/// failure just as before.
///
/// @param path Directory to create.
/// @return true if the directory was newly created, false otherwise.
inline bool makeDirectory(const std::string& path) {
    std::error_code ec;
    const bool created = std::filesystem::create_directory(path, ec);
    return created && !ec;
}

}  // namespace platform
