#ifndef TOOLS__CONFIG_HPP
#define TOOLS__CONFIG_HPP

#include <unistd.h>

#include <filesystem>
#include <string>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace tools
{

/// 返回可执行文件所在目录（Linux: /proc/self/exe）
inline std::filesystem::path executable_dir()
{
  char buffer[4096];
  const auto length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (length <= 0) return std::filesystem::current_path();
  buffer[length] = '\0';
  return std::filesystem::path(buffer).parent_path();
}

/// 解析配置文件路径。回退顺序：
///   1. 原样（绝对路径，或相对当前工作目录）
///   2. 相对可执行文件所在目录
///   3. 相对可执行文件所在目录的上一级（源码树布局：build/contest -> <repo>/configs）
inline std::string resolve_config_path(const std::string & path)
{
  namespace fs = std::filesystem;

  std::error_code ec;
  if (fs::exists(path, ec)) return path;

  const auto exe_dir = executable_dir();
  const fs::path candidates[] = {
    exe_dir / path,
    exe_dir.parent_path() / path,
  };

  for (const auto & candidate : candidates) {
    if (fs::exists(candidate, ec)) return candidate.string();
  }

  logger()->error(
    "[Config] Cannot find config file \"{}\".\n"
    "         cwd = {}\n"
    "         exe = {}\n"
    "         请传入正确的配置文件路径，例如：\n"
    "           ./build/contest configs/contest.yaml",
    path, fs::current_path().string(), exe_dir.string());
  exit(1);
}

/// 把 yaml 里的资源路径（模型文件等）解析成绝对路径。
/// 回退顺序：原样 -> 配置文件所在目录 -> 可执行文件目录 -> 可执行文件目录的上一级 -> 当前工作目录
inline std::string resolve_asset_path(
  const std::string & asset, const std::string & config_path)
{
  namespace fs = std::filesystem;

  std::error_code ec;
  if (asset.empty()) return asset;
  if (fs::path(asset).is_absolute() && fs::exists(asset, ec)) return asset;

  const auto config_dir = fs::path(config_path).parent_path();
  const auto exe_dir = executable_dir();
  const fs::path candidates[] = {
    fs::current_path() / asset,
    config_dir / asset,
    exe_dir / asset,
    exe_dir.parent_path() / asset,
  };

  for (const auto & candidate : candidates) {
    if (fs::exists(candidate, ec)) return candidate.string();
  }

  logger()->warn(
    "[Config] Asset \"{}\" not found (looked relative to cwd, {}, {} and {}). "
    "保持原样，交由调用方报错。",
    asset, config_dir.string(), exe_dir.string(), exe_dir.parent_path().string());
  return asset;
}

/// 载入配置并打印关键路径，便于排查"从哪个目录启动才对"
inline YAML::Node load_config(const std::string & path)
{
  const auto resolved = resolve_config_path(path);
  logger()->info("[Config] Loading {}", resolved);
  logger()->info("[Config] cwd = {}", std::filesystem::current_path().string());
  return load(resolved);
}

}  // namespace tools

#endif  // TOOLS__CONFIG_HPP
