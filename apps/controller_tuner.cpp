#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct TuningParams {
  double kAtt;
  double kRep;
  double dSafe;
};

struct RunMetrics {
  double timeToTarget;
  double oscillationRms;
  double oscillationRangeFirst;
  double oscillationRangeLast;
  double oscillationRangeAll;
  double minError;
  double score;
  bool reachedTarget;
  bool hadSamples;
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
  double reachTol = 1.0;
  double oscWeight = 5.0;
  double minErrWeight = 10.0;
  double oscFirstWeight = 4.0;
  double oscLastWeight = 8.0;
  double oscAllWeight = 1.0;
  double oscWindowSec = 5.0;
  int targetDrone = 1;
  int maxRuns = 0;
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

  parseDouble("--kAttMin", &out->kAttMin);
  parseDouble("--kAttMax", &out->kAttMax);
  parseDouble("--kAttStep", &out->kAttStep);
  parseDouble("--kRepMin", &out->kRepMin);
  parseDouble("--kRepMax", &out->kRepMax);
  parseDouble("--kRepStep", &out->kRepStep);
  parseDouble("--dSafeMin", &out->dSafeMin);
  parseDouble("--dSafeMax", &out->dSafeMax);
  parseDouble("--dSafeStep", &out->dSafeStep);
  parseDouble("--simSeconds", &out->simSeconds);
  parseDouble("--targetX", &out->targetX);
  parseDouble("--reachTol", &out->reachTol);
  parseDouble("--oscWeight", &out->oscWeight);
  parseDouble("--minErrWeight", &out->minErrWeight);
  parseDouble("--oscFirstWeight", &out->oscFirstWeight);
  parseDouble("--oscLastWeight", &out->oscLastWeight);
  parseDouble("--oscAllWeight", &out->oscAllWeight);
  parseDouble("--oscWindowSec", &out->oscWindowSec);
  parseInt("--targetDrone", &out->targetDrone);
  parseInt("--maxRuns", &out->maxRuns);
}

RunMetrics EvaluateRun(const TuningParams& params, const CliArgs& args) {
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
    return {args.simSeconds * 10.0, args.simSeconds * 10.0, args.simSeconds * 10.0,
            args.simSeconds * 10.0, args.simSeconds * 10.0, args.simSeconds * 10.0,
            std::numeric_limits<double>::infinity(), false, false};
  }

  char buffer[4096];
  while (fgets(buffer, sizeof(buffer), pipe)) {
    (void)buffer;
  }
  pclose(pipe);

  std::vector<std::pair<double, double>> samples;
  std::ifstream csv(csvPath);
  if (csv.good()) {
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
      int droneId = std::stoi(cells[1]);
      if (droneId != args.targetDrone) {
        continue;
      }
      double t = std::stod(cells[0]);
      double x = std::stod(cells[4]);
      samples.emplace_back(t, x);
    }
  }

  if (samples.empty()) {
    double penalty = args.simSeconds * 10.0;
    return {penalty, penalty, penalty, penalty, penalty, penalty, penalty, false, false};
  }

  bool reached = false;
  double tReach = std::numeric_limits<double>::infinity();
  double minErr = std::numeric_limits<double>::infinity();
  double tMinErr = args.simSeconds;
  double tStart = samples.front().first;
  double tEnd = samples.back().first;
  for (const auto& s : samples) {
    double err = std::fabs(s.second - args.targetX);
    if (err < minErr) {
      minErr = err;
      tMinErr = s.first;
    }
    if (err <= args.reachTol && !reached) {
      tReach = s.first;
      reached = true;
    }
  }

  const double evalStart = reached ? tReach : tMinErr;
  double sumSq = 0.0;
  int count = 0;
  for (const auto& s : samples) {
    if (s.first + 1e-9 >= evalStart) {
      double err = s.second - args.targetX;
      sumSq += err * err;
      ++count;
    }
  }

  double rms = 0.0;
  if (count > 0) {
    rms = std::sqrt(sumSq / static_cast<double>(count));
  }

  const double firstWindowEnd = tStart + args.oscWindowSec;
  const double lastWindowStart = std::max(tEnd - args.oscWindowSec, tStart);
  double firstMin = std::numeric_limits<double>::infinity();
  double firstMax = -std::numeric_limits<double>::infinity();
  double lastMin = std::numeric_limits<double>::infinity();
  double lastMax = -std::numeric_limits<double>::infinity();
  double allMin = std::numeric_limits<double>::infinity();
  double allMax = -std::numeric_limits<double>::infinity();

  for (const auto& s : samples) {
    const double x = s.second;
    if (x < allMin) {
      allMin = x;
    }
    if (x > allMax) {
      allMax = x;
    }
    if (s.first <= firstWindowEnd) {
      if (x < firstMin) {
        firstMin = x;
      }
      if (x > firstMax) {
        firstMax = x;
      }
    }
    if (s.first + 1e-9 >= lastWindowStart) {
      if (x < lastMin) {
        lastMin = x;
      }
      if (x > lastMax) {
        lastMax = x;
      }
    }
  }

  const double firstRange = (firstMin <= firstMax) ? (firstMax - firstMin) : 0.0;
  const double lastRange = (lastMin <= lastMax) ? (lastMax - lastMin) : 0.0;
  const double allRange = (allMin <= allMax) ? (allMax - allMin) : 0.0;

  const double timeTerm = reached ? tReach : tMinErr + args.simSeconds;
  double score = timeTerm
                 + args.oscWeight * rms
                 + args.minErrWeight * minErr
                 + args.oscFirstWeight * firstRange
                 + args.oscLastWeight * lastRange
                 + args.oscAllWeight * allRange;
  const double timeToTarget = reached ? tReach : -1.0;
  return {timeToTarget, rms, firstRange, lastRange, allRange, minErr, score, reached, true};
}

int main(int argc, char* argv[]) {
  CliArgs args;
  ParseCli(argc, argv, &args);

  TuningParams best{0.0, 0.0, 0.0};
  RunMetrics bestMetrics{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), false, false};
  RunMetrics bestFirstOsc = bestMetrics;
  RunMetrics bestLastOsc = bestMetrics;
  RunMetrics bestAllOsc = bestMetrics;
  RunMetrics bestTime = bestMetrics;

  int runCount = 0;
  for (double kAtt = args.kAttMin; kAtt <= args.kAttMax + 1e-9; kAtt += args.kAttStep) {
    for (double kRep = args.kRepMin; kRep <= args.kRepMax + 1e-9; kRep += args.kRepStep) {
      for (double dSafe = args.dSafeMin; dSafe <= args.dSafeMax + 1e-9; dSafe += args.dSafeStep) {
        if (args.maxRuns > 0 && runCount >= args.maxRuns) {
          break;
        }
        ++runCount;
        TuningParams params{kAtt, kRep, dSafe};
        RunMetrics metrics = EvaluateRun(params, args);

        std::cout << "[Tuning] kAtt=" << params.kAtt
          << " kRep=" << params.kRep
          << " dSafe=" << params.dSafe
          << " timeToTarget=" << metrics.timeToTarget
          << " oscRms=" << metrics.oscillationRms
          << " oscFirstRange=" << metrics.oscillationRangeFirst
          << " oscLastRange=" << metrics.oscillationRangeLast
          << " oscAllRange=" << metrics.oscillationRangeAll
          << " minErr=" << metrics.minError
          << " score=" << metrics.score
          << " reached=" << (metrics.reachedTarget ? 1 : 0)
          << std::endl;

        if (metrics.score < bestMetrics.score) {
          best = params;
          bestMetrics = metrics;
        }
        if (!bestFirstOsc.hadSamples || metrics.oscillationRangeFirst < bestFirstOsc.oscillationRangeFirst) {
          bestFirstOsc = metrics;
        }
        if (!bestLastOsc.hadSamples || metrics.oscillationRangeLast < bestLastOsc.oscillationRangeLast) {
          bestLastOsc = metrics;
        }
        if (!bestAllOsc.hadSamples || metrics.oscillationRangeAll < bestAllOsc.oscillationRangeAll) {
          bestAllOsc = metrics;
        }
        if (!bestTime.hadSamples || (metrics.timeToTarget >= 0.0 && (bestTime.timeToTarget < 0.0 || metrics.timeToTarget < bestTime.timeToTarget))) {
          bestTime = metrics;
        }
      }
    }
  }

  std::cout << "[Best] kAtt=" << best.kAtt
            << " kRep=" << best.kRep
            << " dSafe=" << best.dSafe
            << " timeToTarget=" << bestMetrics.timeToTarget
            << " oscRms=" << bestMetrics.oscillationRms
            << " oscFirstRange=" << bestMetrics.oscillationRangeFirst
            << " oscLastRange=" << bestMetrics.oscillationRangeLast
            << " oscAllRange=" << bestMetrics.oscillationRangeAll
            << " minErr=" << bestMetrics.minError
            << " score=" << bestMetrics.score
            << " reached=" << (bestMetrics.reachedTarget ? 1 : 0)
            << std::endl;

  std::cout << "[BestFirstOsc] oscFirstRange=" << bestFirstOsc.oscillationRangeFirst
            << " timeToTarget=" << bestFirstOsc.timeToTarget
            << " oscLastRange=" << bestFirstOsc.oscillationRangeLast
            << " oscAllRange=" << bestFirstOsc.oscillationRangeAll
            << " minErr=" << bestFirstOsc.minError
            << std::endl;

  std::cout << "[BestLastOsc] oscLastRange=" << bestLastOsc.oscillationRangeLast
            << " timeToTarget=" << bestLastOsc.timeToTarget
            << " oscFirstRange=" << bestLastOsc.oscillationRangeFirst
            << " oscAllRange=" << bestLastOsc.oscillationRangeAll
            << " minErr=" << bestLastOsc.minError
            << std::endl;

  std::cout << "[BestAllOsc] oscAllRange=" << bestAllOsc.oscillationRangeAll
            << " timeToTarget=" << bestAllOsc.timeToTarget
            << " oscFirstRange=" << bestAllOsc.oscillationRangeFirst
            << " oscLastRange=" << bestAllOsc.oscillationRangeLast
            << " minErr=" << bestAllOsc.minError
            << std::endl;

  std::cout << "[BestTime] timeToTarget=" << bestTime.timeToTarget
            << " oscFirstRange=" << bestTime.oscillationRangeFirst
            << " oscLastRange=" << bestTime.oscillationRangeLast
            << " oscAllRange=" << bestTime.oscillationRangeAll
            << " minErr=" << bestTime.minError
            << std::endl;

  return 0;
}
