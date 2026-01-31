#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct TuningParams {
  double kAtt;
  double kRep;
  double dSafe;
};

struct DroneOscillationStats {
  int droneId = 0;
  std::vector<double> oscillationsX;
  std::vector<double> oscillationsY;
  std::vector<double> oscillationsZ;
  double firstOscX = 0.0;
  double lastOscX = 0.0;
  double avgOscX = 0.0;
  bool strictlyReducingX = false;
  double firstOscY = 0.0;
  double lastOscY = 0.0;
  double avgOscY = 0.0;
  bool strictlyReducingY = false;
  double firstOscZ = 0.0;
  double lastOscZ = 0.0;
  double avgOscZ = 0.0;
  bool strictlyReducingZ = false;
  double firstOsc = 0.0;
  double lastOsc = 0.0;
  double avgOsc = 0.0;
  bool strictlyReducing = false;
  bool hasOscillations = false;
  double minDistanceToOthers = -1.0;
  double timeToTarget = -1.0;
};

struct RunMetrics {
  double avgFirstOsc = 0.0;
  double avgLastOsc = 0.0;
  double avgAvgOsc = 0.0;
  double avgMinDistance = -1.0;
  double avgTimeToTarget = -1.0;
  int dronesWithOscillations = 0;
  int totalDrones = 0;
  bool allStrictlyReducing = false;
  double score = std::numeric_limits<double>::infinity();
  bool hadSamples = false;
};

struct CliArgs {
  double kAttMin = 0.5;
  double kAttMax = 2.0;
  double kAttStep = 0.5;
  double kRepMin = 1.0;
  double kRepMax = 8.0;
  double kRepStep = 1.0;
  double dSafeMin = 1.0;
  double dSafeMax = 8.0;
  double dSafeStep = 1.0;
  double simSeconds = 600.0;
  double targetX = 30.0;
  double targetY = 7.5;
  double targetZ = 0.0;
  int lostDroneId = 1;  // Drone 1 is the lost drone (sends HELP_PROXY)
};

bool TryGetArgValue(const std::vector<std::string>& args, const std::string& key, std::string* value) {
  const std::string prefix = key + "=";
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i].rfind(prefix, 0) == 0) {
      *value = args[i].substr(prefix.size());
      return true;
    }
    if (args[i] == key && i + 1 < args.size()) {
      *value = args[i + 1];
      return true;
    }
  }
  return false;
}

void ParseCli(int argc, char* argv[], CliArgs* out) {
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  auto parseDouble = [&](const std::string& key, double* dst) {
    std::string value;
    if (TryGetArgValue(args, key, &value)) {
      *dst = std::stod(value);
    }
  };

  auto parseInt = [&](const std::string& key, int* dst) {
    std::string value;
    if (TryGetArgValue(args, key, &value)) {
      *dst = std::stoi(value);
    }
  };

  parseDouble("--simSeconds", &out->simSeconds);
  parseDouble("--targetX", &out->targetX);
  parseDouble("--targetY", &out->targetY);
  parseDouble("--targetZ", &out->targetZ);
  parseDouble("--kAttMin", &out->kAttMin);
  parseDouble("--kAttMax", &out->kAttMax);
  parseDouble("--kAttStep", &out->kAttStep);
  parseDouble("--kRepMin", &out->kRepMin);
  parseDouble("--kRepMax", &out->kRepMax);
  parseDouble("--kRepStep", &out->kRepStep);
  parseDouble("--dSafeMin", &out->dSafeMin);
  parseDouble("--dSafeMax", &out->dSafeMax);
  parseDouble("--dSafeStep", &out->dSafeStep);
  parseInt("--lostDroneId", &out->lostDroneId);
}

struct PositionSample {
  double t;
  int droneId;
  double x;
  double y;
  double z;
};

static std::vector<PositionSample> LoadPositionSamples(const std::string& csvPath) {
  std::vector<PositionSample> samples;
  std::ifstream csv(csvPath);
  if (!csv.good()) {
    return samples;
  }

  std::string line;
  bool first = true;
  while (std::getline(csv, line)) {
    if (first) {
      first = false;
      continue;
    }
    if (line.empty()) {
      continue;
    }
    std::istringstream ss(line);
    std::string cell;
    std::vector<std::string> cells;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (cells.size() < 7) {
      continue;
    }
    PositionSample sample;
    sample.t = std::stod(cells[0]);
    sample.droneId = std::stoi(cells[1]);
    sample.x = std::stod(cells[4]);
    sample.y = std::stod(cells[5]);
    sample.z = std::stod(cells[6]);
    samples.push_back(sample);
  }
  return samples;
}

static int SignWithEps(double v, double eps) {
  if (v > eps) {
    return 1;
  }
  if (v < -eps) {
    return -1;
  }
  return 0;
}

static std::vector<double> ComputeAxisOscillations(const std::vector<double>& values) {
  const double eps = 1e-9;
  std::vector<double> turningPoints;
  int lastDirSign = 0;
  for (size_t i = 1; i < values.size(); ++i) {
    const double dv = values[i] - values[i - 1];
    const int dirSign = SignWithEps(dv, eps);
    if (dirSign == 0) {
      continue;
    }
    if (lastDirSign == 0) {
      lastDirSign = dirSign;
      continue;
    }
    if (dirSign != lastDirSign) {
      turningPoints.push_back(values[i - 1]);
      lastDirSign = dirSign;
    }
  }

  std::vector<double> oscillations;
  for (size_t i = 1; i < turningPoints.size(); ++i) {
    const double v1 = turningPoints[i - 1];
    const double v2 = turningPoints[i];
    oscillations.push_back(std::fabs(v2 - v1));
  }
  return oscillations;
}

static std::vector<DroneOscillationStats> ComputeDroneOscillationStats(
  const std::vector<PositionSample>& samples,
  double targetX,
  double targetY,
  double targetZ,
  double reachTol,
  int lostDroneId
) {
  const double eps = 1e-9;
  std::unordered_map<int, std::vector<PositionSample>> droneSamples;
  for (const auto& s : samples) {
    if (lostDroneId >= 0 && s.droneId == lostDroneId) {
      continue;
    }
    droneSamples[s.droneId].push_back(s);
  }

  std::vector<DroneOscillationStats> stats;
  stats.reserve(droneSamples.size());

  for (const auto& entry : droneSamples) {
    const int droneId = entry.first;
    std::vector<PositionSample> samplesSorted = entry.second;
    std::sort(samplesSorted.begin(), samplesSorted.end(),
      [](const auto& a, const auto& b) { return a.t < b.t; });
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(samplesSorted.size());
    ys.reserve(samplesSorted.size());
    zs.reserve(samplesSorted.size());
    for (const auto& s : samplesSorted) {
      xs.push_back(s.x);
      ys.push_back(s.y);
      zs.push_back(s.z);
    }
    DroneOscillationStats s;
    s.droneId = droneId;

    if (xs.size() < 3) {
      stats.push_back(s);
      continue;
    }

    s.oscillationsX = ComputeAxisOscillations(xs);
    s.oscillationsY = ComputeAxisOscillations(ys);
    s.oscillationsZ = ComputeAxisOscillations(zs);

    auto computeAxisStats = [&](const std::vector<double>& osc, double* first, double* last,
                                double* avg, bool* reducing) {
      if (osc.empty()) {
        *first = 0.0;
        *last = 0.0;
        *avg = 0.0;
        *reducing = false;
        return;
      }
      *first = osc.front();
      *last = osc.back();
      double sum = 0.0;
      for (double r : osc) {
        sum += r;
      }
      *avg = sum / static_cast<double>(osc.size());
      *reducing = true;
      for (size_t i = 1; i < osc.size(); ++i) {
        if (!(osc[i] < osc[i - 1] - eps)) {
          *reducing = false;
          break;
        }
      }
    };

    computeAxisStats(s.oscillationsX, &s.firstOscX, &s.lastOscX, &s.avgOscX, &s.strictlyReducingX);
    computeAxisStats(s.oscillationsY, &s.firstOscY, &s.lastOscY, &s.avgOscY, &s.strictlyReducingY);
    computeAxisStats(s.oscillationsZ, &s.firstOscZ, &s.lastOscZ, &s.avgOscZ, &s.strictlyReducingZ);

    s.hasOscillations = !s.oscillationsX.empty() || !s.oscillationsY.empty() || !s.oscillationsZ.empty();
    s.strictlyReducing = s.hasOscillations
      && (s.oscillationsX.empty() || s.strictlyReducingX)
      && (s.oscillationsY.empty() || s.strictlyReducingY)
      && (s.oscillationsZ.empty() || s.strictlyReducingZ);

    s.firstOsc = (s.firstOscX + s.firstOscY + s.firstOscZ) / 3.0;
    s.lastOsc = (s.lastOscX + s.lastOscY + s.lastOscZ) / 3.0;
    s.avgOsc = (s.avgOscX + s.avgOscY + s.avgOscZ) / 3.0;

    for (const auto& sample : samplesSorted) {
      const double dx = sample.x - targetX;
      const double dy = sample.y - targetY;
      const double dz = sample.z - targetZ;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist <= reachTol) {
        s.timeToTarget = sample.t;
        break;
      }
    }

    stats.push_back(s);
  }

  return stats;
}

static void ComputeMinDistancesToOthers(
  const std::vector<PositionSample>& samples,
  std::vector<DroneOscillationStats>* stats,
  int lostDroneId
) {
  if (!stats || stats->empty() || samples.empty()) {
    return;
  }

  struct Vec3 {
    double x;
    double y;
    double z;
  };

  std::unordered_map<int, size_t> idToIndex;
  for (size_t i = 0; i < stats->size(); ++i) {
    idToIndex[(*stats)[i].droneId] = i;
    (*stats)[i].minDistanceToOthers = std::numeric_limits<double>::infinity();
  }

  // Track positions of ALL drones (including lost drone) for distance computation
  // We want to know if any repositioning drone gets too close to ANY other drone
  std::unordered_map<int, Vec3> lastPositions;

  for (const auto& s : samples) {
    const int id = s.droneId;
    // DON'T filter by lostDroneId here - we need all positions for distance calculation
    lastPositions[id] = {s.x, s.y, s.z};

    if (lastPositions.size() < 2) {
      continue;
    }

    // Compute distances between current drone and all other tracked drones
    for (const auto& other : lastPositions) {
      if (other.first == id) {
        continue;
      }
      const double dx = s.x - other.second.x;
      const double dy = s.y - other.second.y;
      const double dz = s.z - other.second.z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Update min distance for drones that are in our stats
      auto idxA = idToIndex.find(id);
      auto idxB = idToIndex.find(other.first);
      if (idxA != idToIndex.end() && dist < (*stats)[idxA->second].minDistanceToOthers) {
        (*stats)[idxA->second].minDistanceToOthers = dist;
      }
      if (idxB != idToIndex.end() && dist < (*stats)[idxB->second].minDistanceToOthers) {
        (*stats)[idxB->second].minDistanceToOthers = dist;
      }
    }
  }

  for (auto& s : *stats) {
    if (std::isinf(s.minDistanceToOthers)) {
      s.minDistanceToOthers = -1.0;
    }
  }
}

RunMetrics EvaluateRun(
  const TuningParams& params,
  const CliArgs& args,
  std::vector<DroneOscillationStats>* outDroneStats
) {
  const double reachTol = 2.0;
  const std::string csvPath = "/project/output/tuning_run.csv";
  std::ostringstream cmd;
  cmd << "/project/build-docker/swarm_demo_sim1"
      << " --simSeconds=" << args.simSeconds
      << " --kAtt=" << params.kAtt
      << " --kRep=" << params.kRep
      << " --dSafe=" << params.dSafe
      << " --csvOut=" << csvPath
      << " --animOut=/project/output/drone-simulation.xml";

  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) {
    return {};
  }

  char buffer[4096];
  while (fgets(buffer, sizeof(buffer), pipe)) {
    (void)buffer;
  }
  pclose(pipe);

  std::vector<PositionSample> positionSamples = LoadPositionSamples(csvPath);
  if (positionSamples.empty()) {
    return {};
  }

  std::vector<DroneOscillationStats> droneStats = ComputeDroneOscillationStats(
    positionSamples,
    args.targetX,
    args.targetY,
    args.targetZ,
    reachTol,
    args.lostDroneId
  );
  ComputeMinDistancesToOthers(positionSamples, &droneStats, args.lostDroneId);
  if (outDroneStats) {
    *outDroneStats = droneStats;
  }

  int totalDrones = static_cast<int>(droneStats.size());
  int dronesWithOsc = 0;
  double sumFirst = 0.0;
  double sumAvg = 0.0;
  double sumLast = 0.0;
  double sumMinDistance = 0.0;
  int dronesWithDistance = 0;
  double sumTimeToTarget = 0.0;
  int dronesWithTime = 0;
  bool allReducing = true;

  for (const auto& s : droneStats) {
    if (s.hasOscillations) {
      ++dronesWithOsc;
      sumFirst += s.firstOsc;
      sumAvg += s.avgOsc;
      sumLast += s.lastOsc;
      if (!s.strictlyReducing) {
        allReducing = false;
      }
    } else {
      allReducing = false;
    }
    if (s.minDistanceToOthers >= 0.0) {
      sumMinDistance += s.minDistanceToOthers;
      ++dronesWithDistance;
    }
    if (s.timeToTarget >= 0.0) {
      sumTimeToTarget += s.timeToTarget;
      ++dronesWithTime;
    }
  }

  RunMetrics metrics;
  metrics.totalDrones = totalDrones;
  metrics.dronesWithOscillations = dronesWithOsc;
  metrics.allStrictlyReducing = (dronesWithOsc > 0) && allReducing;
  if (dronesWithOsc > 0) {
    const double denom = static_cast<double>(dronesWithOsc);
    metrics.avgFirstOsc = sumFirst / denom;
    metrics.avgAvgOsc = sumAvg / denom;
    metrics.avgLastOsc = sumLast / denom;
  }
  if (dronesWithDistance > 0) {
    metrics.avgMinDistance = sumMinDistance / static_cast<double>(dronesWithDistance);
  }
  if (dronesWithTime > 0) {
    metrics.avgTimeToTarget = sumTimeToTarget / static_cast<double>(dronesWithTime);
  }
  metrics.hadSamples = true;

  metrics.score = metrics.avgFirstOsc + metrics.avgAvgOsc + metrics.avgLastOsc;
  if (dronesWithOsc == 0) {
    metrics.score += 1e6;
  }
  if (metrics.avgMinDistance < 0.0) {
    metrics.score += 1e5;
  }

  return metrics;
}

int main(int argc, char* argv[]) {
  CliArgs args;
  ParseCli(argc, argv, &args);

  TuningParams best{0.0, 0.0, 0.0};
  RunMetrics bestMetrics;

  int runCount = 0;
  for (double kAtt = args.kAttMin; kAtt <= args.kAttMax + 1e-9; kAtt += args.kAttStep) {
    for (double kRep = args.kRepMin; kRep <= args.kRepMax + 1e-9; kRep += args.kRepStep) {
      for (double dSafe = args.dSafeMin; dSafe <= args.dSafeMax + 1e-9; dSafe += args.dSafeStep) {
        ++runCount;
        TuningParams params{kAtt, kRep, dSafe};
        std::vector<DroneOscillationStats> droneStats;
        RunMetrics metrics = EvaluateRun(params, args, &droneStats);

        std::cout << "[Tuning] kAtt=" << params.kAtt
          << " kRep=" << params.kRep
          << " dSafe=" << params.dSafe
          << " avgFirstOsc=" << metrics.avgFirstOsc
          << " avgOsc=" << metrics.avgAvgOsc
          << " avgLastOsc=" << metrics.avgLastOsc
          << " dronesWithOsc=" << metrics.dronesWithOscillations
          << " totalDrones=" << metrics.totalDrones
          << " allReducing=" << (metrics.allStrictlyReducing ? 1 : 0)
          << " avgMinDist=" << metrics.avgMinDistance
          << " avgTimeToTarget=" << metrics.avgTimeToTarget
          << " score=" << metrics.score
          << std::endl;

        for (const auto& s : droneStats) {
          std::cout << "[DroneOsc] id=" << s.droneId
            << " firstOsc=" << s.firstOsc
            << " avgOsc=" << s.avgOsc
            << " lastOsc=" << s.lastOsc
            << " count=" << (s.oscillationsX.size() + s.oscillationsY.size() + s.oscillationsZ.size())
            << " reducing=" << (s.strictlyReducing ? 1 : 0)
            << " minDist=" << s.minDistanceToOthers
            << " timeToTarget=" << s.timeToTarget
            << " firstOscX=" << s.firstOscX
            << " avgOscX=" << s.avgOscX
            << " lastOscX=" << s.lastOscX
            << " firstOscY=" << s.firstOscY
            << " avgOscY=" << s.avgOscY
            << " lastOscY=" << s.lastOscY
            << " firstOscZ=" << s.firstOscZ
            << " avgOscZ=" << s.avgOscZ
            << " lastOscZ=" << s.lastOscZ
            << std::endl;
        }

        if (metrics.score < bestMetrics.score) {
          best = params;
          bestMetrics = metrics;
        }
      }
    }
  }

  std::cout << "[Best] kAtt=" << best.kAtt
            << " kRep=" << best.kRep
            << " dSafe=" << best.dSafe
            << " avgFirstOsc=" << bestMetrics.avgFirstOsc
            << " avgOsc=" << bestMetrics.avgAvgOsc
            << " avgLastOsc=" << bestMetrics.avgLastOsc
            << " dronesWithOsc=" << bestMetrics.dronesWithOscillations
            << " totalDrones=" << bestMetrics.totalDrones
            << " allReducing=" << (bestMetrics.allStrictlyReducing ? 1 : 0)
            << " avgMinDist=" << bestMetrics.avgMinDistance
            << " avgTimeToTarget=" << bestMetrics.avgTimeToTarget
            << " score=" << bestMetrics.score
            << std::endl;

  return 0;
}
