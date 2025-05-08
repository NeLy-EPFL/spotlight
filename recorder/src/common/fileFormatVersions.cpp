#include "fileFormatVersions.hpp"

bool checkVersionCompatibility(
    int major, int minor, int specifiedMajor, int specifiedMinor)
{
    if (major == specifiedMajor && minor >= specifiedMinor)
    {
        return true;
    }
    spdlog::error(
        "Version mismatch: found {}.{} but required "
        "major version {} and minor version >={}",
        major, minor, specifiedMajor, specifiedMinor);
    return false;
}