#include "asm_socketcan_bridge.h"

#include <cstdio>
#include <filesystem>

namespace asm_socketcan_bridge {

  namespace {

    std::int64_t steadyNowNs()
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // Command IDs the bridge reader consumes (0x578-0x57E and 1450).
    bool isCommandFrameId(std::uint32_t id)
    {
      return (id >= 1400 && id <= 1406) || id == 1450;
    }

    constexpr std::uint32_t kMinStepMs = 1;
    constexpr std::uint32_t kMaxStepMs = 100;
    constexpr std::uint32_t kDefaultStepMs = 10;
    constexpr std::int64_t kDefaultTimeoutMs = 20;
    constexpr std::int64_t kMaxTimeoutMs = 10000;

  }  // namespace

  void AsmSocketCanBridgeNode::configureSimStepping()
  {
    this->simStepMarkerEnabled_ = this->declare_parameter<bool>("sim.step_marker.enabled", true);
    this->readinessMinClockSubscribers_ =
      std::max<std::int64_t>(0, this->declare_parameter<int64_t>("sim.readiness.min_clock_subscribers", 1));
    this->readinessSettleMs_ =
      std::max<std::int64_t>(0, this->declare_parameter<int64_t>("sim.readiness.settle_ms", 1000));
    const int64_t static_step_ms = this->declare_parameter<int64_t>("sim.static.step_ms", kDefaultStepMs);
    const auto required_ids = this->declare_parameter<std::vector<int64_t>>(
      "sim.static.required_command_ids", std::vector<int64_t>{1400, 1401, 1402, 1403, 1404});
    const auto command_periods_ms = this->declare_parameter<std::vector<int64_t>>(
      "sim.static.command_periods_ms", std::vector<int64_t>{10, 10, 10, 10, 500});
    const auto command_first_due_ms = this->declare_parameter<std::vector<int64_t>>(
      "sim.static.command_first_due_ms", std::vector<int64_t>{10, 10, 10, 10, 510});
    const int64_t configured_timeout_ms =
      this->declare_parameter<int64_t>("sim.timeout_ms", kDefaultTimeoutMs);
    this->logSimSteps_ = this->declare_parameter<bool>("logging.sim_steps", false);
    this->logRealTimeFactor_ = this->declare_parameter<bool>("logging.real_time_factor", false);
    this->logSimObservability_ = this->declare_parameter<bool>("logging.sim_observability", false);
    const std::string replay_file = this->declare_parameter<std::string>("sim.replay_file", "");
    const std::string adaptation_mode =
      this->declare_parameter<std::string>("adaptation.mode", "on");
    this->adaptationDurationS_ = this->declare_parameter<double>("adaptation.duration_s", 3.0);
    this->adaptationRealtimeFactor_ =
      this->declare_parameter<double>("adaptation.realtime_factor", 1.0);
    this->adaptationConfig_.min_step_ms = kMinStepMs;
    this->adaptationConfig_.max_step_ms = kMaxStepMs;
    this->adaptationConfig_.default_step_ms = kDefaultStepMs;
    this->adaptationEnabled_ = adaptation_mode != "off";
    if (adaptation_mode != "on" && adaptation_mode != "off") {
      RCLCPP_WARN(this->get_logger(), "Invalid adaptation.mode '%s'; using 'on'",
                  adaptation_mode.c_str());
    }
    if (this->adaptationDurationS_ < 0.0 || this->adaptationDurationS_ > 600.0) {
      RCLCPP_WARN(this->get_logger(), "adaptation.duration_s out of range (%.3f); using 3.0",
                  this->adaptationDurationS_);
      this->adaptationDurationS_ = 3.0;
    }
    if (!(this->adaptationRealtimeFactor_ > 0.0)) {
      RCLCPP_WARN(this->get_logger(), "adaptation.realtime_factor must be positive (%.3f); using 1.0",
                  this->adaptationRealtimeFactor_);
      this->adaptationRealtimeFactor_ = 1.0;
    }
    this->adaptationConfig_.wall_step_ns =
      std::max<std::int64_t>(1, static_cast<std::int64_t>(1e6 / this->adaptationRealtimeFactor_));

    if (!this->simModeEnabled) {
      if (!replay_file.empty()) {
        RCLCPP_WARN(this->get_logger(), "sim.replay_file is ignored in real-time mode.");
      }
      return;
    }

    if (static_step_ms < kMinStepMs || static_step_ms > kMaxStepMs) {
      RCLCPP_WARN(this->get_logger(),
                  "sim.static.step_ms out of range (%lld); accepted are %u-%u ms; using %u",
                  static_cast<long long>(static_step_ms), kMinStepMs, kMaxStepMs, kDefaultStepMs);
      this->sim_step_ms_ = kDefaultStepMs;
    } else {
      this->sim_step_ms_ = static_cast<std::uint32_t>(static_step_ms);
    }
    std::int64_t timeout_ms = configured_timeout_ms;
    if (timeout_ms < 1 || timeout_ms > kMaxTimeoutMs) {
      RCLCPP_WARN(this->get_logger(),
                  "sim.timeout_ms out of range (%lld); accepted are 1-%lld ms; using %lld",
                  static_cast<long long>(configured_timeout_ms), static_cast<long long>(kMaxTimeoutMs),
                  static_cast<long long>(kDefaultTimeoutMs));
      timeout_ms = kDefaultTimeoutMs;
    }

    StepCoordinator::Config config;
    config.step_ms = this->sim_step_ms_;
    config.timeout_ns = timeout_ms * 1000000;
    std::string id_list;
    for (std::size_t index = 0; index < required_ids.size(); ++index) {
      const auto id = required_ids[index];
      if (id < 0 || id > 0x7FF) {
        RCLCPP_WARN(this->get_logger(),
                    "Ignoring sim.static.required_command_ids entry %lld (not a standard CAN ID)",
                    static_cast<long long>(id));
        continue;
      }
      const auto can_id = static_cast<std::uint32_t>(id);
      if (std::find(staticRequiredCommandIds_.begin(), staticRequiredCommandIds_.end(), can_id) !=
          staticRequiredCommandIds_.end()) {
        continue;
      }
      std::uint32_t period_ms = this->sim_step_ms_;
      if (index < command_periods_ms.size() && command_periods_ms[index] >= this->sim_step_ms_ &&
          command_periods_ms[index] <= kMaxTimeoutMs) {
        period_ms = static_cast<std::uint32_t>(command_periods_ms[index]);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "sim.static.command_periods_ms has no valid entry for command ID %u; using the step (%u ms)",
                    can_id, this->sim_step_ms_);
      }
      staticRequiredCommandIds_.push_back(can_id);
      std::uint32_t first_due_ms = period_ms;
      if (index < command_first_due_ms.size() && command_first_due_ms[index] >= 0 &&
          command_first_due_ms[index] <= kMaxTimeoutMs) {
        first_due_ms = static_cast<std::uint32_t>(command_first_due_ms[index]);
      }
      config.schedule.push_back({can_id, period_ms, first_due_ms});
      id_list += (id_list.empty() ? "" : ",") + std::to_string(can_id) + "@" + std::to_string(period_ms);
    }
    this->stepCoordinator_.emplace(std::move(config));
    this->activeTimeoutMs_ = timeout_ms;
    if (!replay_file.empty()) {
      // Recorded commands replace the stack: no schedule, no adaptation, no wait for CAN intake.
      std::ifstream input(replay_file);
      std::string error = "cannot open file";
      if (!input.is_open() || !loadReplay(input, this->replaySteps_, error)) {
        throw std::runtime_error("sim.replay_file '" + replay_file + "': " + error);
      }
      StepCoordinator::Config replaying;
      replaying.step_ms = this->sim_step_ms_;
      replaying.timeout_ns = timeout_ms * 1000000;
      this->stepCoordinator_.emplace(std::move(replaying));
      this->replayActive_ = true;
      this->adaptationEnabled_ = false;
      const auto schedule = replaySchedule(this->replaySteps_);
      RCLCPP_INFO(this->get_logger(),
                  "Simulation stepping owned by the environment: open-loop replay of %zu steps (%llu ms) "
                  "from %s; command intake from CAN is disabled, marker=%s",
                  this->replaySteps_.size(),
                  static_cast<unsigned long long>(this->replaySteps_.back().t_ms),
                  replay_file.c_str(), this->simStepMarkerEnabled_ ? "on" : "off");
      RCLCPP_INFO(this->get_logger(), "SIM_STEP replay schedule: step_ms=%u switch_ms=%llu",
                  schedule.step_ms, static_cast<unsigned long long>(schedule.switch_ms));
      return;
    }
    if (this->adaptationEnabled_) {
      // The schedule is learned from 1 ms steps; frames are accepted in every state meanwhile.
      StepCoordinator::Config adapting;
      adapting.step_ms = 1;
      adapting.timeout_ns = timeout_ms * 1000000;
      adapting.accept_outside_release = true;
      this->stepCoordinator_.emplace(std::move(adapting));
      this->sim_step_ms_ = 1;
      this->adaptationDetector_.emplace(this->adaptationConfig_);
      RCLCPP_INFO(this->get_logger(),
                  "Simulation stepping owned by the environment: adaptation on (%.3f s, realtime_factor %.3f), "
                  "marker=%s",
                  this->adaptationDurationS_, this->adaptationRealtimeFactor_,
                  this->simStepMarkerEnabled_ ? "on" : "off");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Simulation stepping owned by the environment: step=%u ms timeout=%lld ms "
                "required_command_ids=[%s] marker=%s",
                this->sim_step_ms_, static_cast<long long>(timeout_ms), id_list.c_str(),
                this->simStepMarkerEnabled_ ? "on" : "off");
  }

  void AsmSocketCanBridgeNode::buildCommandCounterLookup()
  {
    command_counter_signals_.clear();
    for (const auto &message : can_message_info) {
      if (!isCommandFrameId(message.id)) {
        continue;
      }
      for (const auto &signal : message.signals) {
        if (std::string_view(signal.name).find("counter") != std::string_view::npos) {
          command_counter_signals_.emplace(message.id, &signal);
          break;
        }
      }
    }
  }

  void AsmSocketCanBridgeNode::submitCommandFrame(const struct can_frame &in_frame)
  {
    if (!isCommandFrameId(in_frame.can_id) || !stepCoordinator_ || replayActive_) {
      return;
    }
    CommandFrame frame;
    frame.id = in_frame.can_id;
    frame.dlc = in_frame.can_dlc;
    std::copy_n(in_frame.data, frame.data.size(), frame.data.begin());
    if (const auto it = command_counter_signals_.find(frame.id); it != command_counter_signals_.end()) {
      frame.has_counter = true;
      frame.counter = static_cast<std::uint8_t>(extractBits(frame.data.data(), *it->second));
    }

    bool closed_now = false;
    {
      const std::lock_guard<std::mutex> lock(coordinator_mutex_);
      const bool was_closed = stepCoordinator_->state() == StepState::kClosed;
      const auto now_ns = steadyNowNs();
      const auto disposition = stepCoordinator_->onFrame(frame, now_ns);
      if (adaptationDetector_ && disposition == FrameDisposition::kAccepted) {
        adaptationDetector_->onArrival(frame.id, lastReleaseTimeMs_, now_ns, now_ns - lastReleaseNs_);
      }
      closed_now = !was_closed && stepCoordinator_->state() == StepState::kClosed;
    }
    if (closed_now) {
      coordinator_cv_.notify_all();
    }
  }

  bool AsmSocketCanBridgeNode::waitForSimReadiness()
  {
    // The bridge's own TimeSource subscription is not counted.
    const std::size_t own_subscriptions = 1;
    const auto required = static_cast<std::size_t>(readinessMinClockSubscribers_) + own_subscriptions;
    const auto settle = std::chrono::milliseconds(readinessSettleMs_);
    std::optional<std::chrono::steady_clock::time_point> ready_since;
    auto last_log = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(),
                "SIM_STEP waiting for %lld /clock subscriber(s) and %lld ms settle time.",
                static_cast<long long>(readinessMinClockSubscribers_),
                static_cast<long long>(readinessSettleMs_));
    while (!stop_stepping_.load() && rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (this->simClockTimePublisher_->get_subscription_count() >= required) {
        if (!ready_since) {
          ready_since = now;
        }
        if (now - *ready_since >= settle) {
          break;
        }
      } else {
        ready_since.reset();
        if (now - last_log >= std::chrono::seconds(5)) {
          RCLCPP_WARN(this->get_logger(),
                      "SIM_STEP still waiting for /clock subscribers (%zu of %zu matched).",
                      this->simClockTimePublisher_->get_subscription_count() - own_subscriptions,
                      required - own_subscriptions);
          last_log = now;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (stop_stepping_.load() || !rclcpp::ok()) {
      return false;
    }
    return waitForManeuverActive();
  }

  bool AsmSocketCanBridgeNode::waitForManeuverActive()
  {
    auto last_log = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    while (!stop_stepping_.load() && rclcpp::ok()) {
      {
        std::shared_lock<std::shared_mutex> lock(can_bus_mutex_);
        if (this->maneuverStarted) {
          return true;
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= std::chrono::seconds(5)) {
        RCLCPP_WARN(this->get_logger(), "SIM_STEP waiting for maneuver state 3; stepping is paused.");
        last_log = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      vesiCallback();
    }
    return false;
  }

  void AsmSocketCanBridgeNode::waitForStepClose()
  {
    std::unique_lock<std::mutex> lock(coordinator_mutex_);
    while (!stop_stepping_.load()) {
      if (stepCoordinator_->poll(steadyNowNs())) {
        return;
      }
      const std::chrono::steady_clock::time_point deadline{
        std::chrono::nanoseconds(stepCoordinator_->deadlineNs())};
      coordinator_cv_.wait_until(lock, deadline, [this]() {
        return stop_stepping_.load() || stepCoordinator_->state() == StepState::kClosed;
      });
    }
  }

  void AsmSocketCanBridgeNode::environmentStepLoop()
  {
    if (!waitForSimReadiness()) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "SIM_STEP environment-owned stepping started.");

    // Baseline outputs at t=0; /clock 0 was published at start-up.
    current_sim_step_ = 0;
    canOutputSchedule_.runDue(simTime_.totalMilliseconds());
    rosOutputSchedule_.runDue(simTime_.totalMilliseconds());

    if (replayActive_) {
      runReplay();
    } else {
      bool proceed = true;
      if (adaptationEnabled_) {
        proceed = runAdaptation();
      }
      while (proceed && !stop_stepping_.load() && rclcpp::ok()) {
        if (!waitForManeuverActive() || !runEnvironmentStep()) {
          break;
        }
      }
    }

    const std::lock_guard<std::mutex> lock(coordinator_mutex_);
    if (stepCoordinator_->step() > 0 && stepCoordinator_->state() == StepState::kClosed) {
      logStepRecord(stepCoordinator_->record());
    }
  }

  bool AsmSocketCanBridgeNode::runEnvironmentStep(const ReplayStep *replay)
  {
    {
      StepStart start;
      const auto step_wall_start = steadyNowNs();
      {
        const std::lock_guard<std::mutex> lock(coordinator_mutex_);
        if (replay) {
          // The recorded times carry the step size, including the switch after adaptation.
          auto config = stepCoordinator_->config();
          config.step_ms = static_cast<std::uint32_t>(replay->t_ms - stepCoordinator_->timeMs());
          if (config.step_ms != stepCoordinator_->config().step_ms) {
            stepCoordinator_->reconfigure(std::move(config));
          }
          sim_step_ms_ = stepCoordinator_->config().step_ms;
        }
        start = stepCoordinator_->beginStep();
      }
      if (replay) {
        start.snapshot.clear();
        for (const auto &frame : replay->frames) {
          start.snapshot.push_back({frame, true});
        }
      }
      if (start.finalized) {
        logStepRecord(*start.finalized);
      }
      logAppliedCommands(start.step, start.t_ms, start.snapshot);
      for (const auto &entry : start.snapshot) {
        if (!entry.fresh) {
          continue;
        }
        struct can_frame frame{};
        frame.can_id = entry.frame.id;
        frame.can_dlc = entry.frame.dlc;
        std::copy(entry.frame.data.begin(), entry.frame.data.end(), frame.data);
        applyCommandFrame(frame);
      }
      latchCommandSnapshot();

      const auto substeps_before = sim_substeps_completed_.load();
      const auto step = start.step;
      const bool completed = runSimStepSequence(
        static_cast<std::uint16_t>(sim_step_ms_),
        [this]() {
          this->vesiCallback();
          sim_substeps_completed_.fetch_add(1);
        },
        [this, step]() {
          {
            const std::lock_guard<std::mutex> lock(coordinator_mutex_);
            stepCoordinator_->markPublishing();
          }
          current_sim_step_ = step;
          canOutputSchedule_.runDue(simTime_.totalMilliseconds());
        },
        [this]() { rosOutputSchedule_.runDue(simTime_.totalMilliseconds()); },
        [this, step]() {return !simStepMarkerEnabled_ || publishSimStepMarker(step);},
        [this]() {
          // Release before publishing so a fast stack's frames already fall into the window.
          {
            const std::lock_guard<std::mutex> lock(coordinator_mutex_);
            stepCoordinator_->release(steadyNowNs());
            lastReleaseNs_ = steadyNowNs();
            lastReleaseTimeMs_ = stepCoordinator_->timeMs();
          }
          this->simClockTimeCallback();
        });

      if (sim_substeps_completed_.load() - substeps_before != sim_step_ms_) {
        sim_substep_mismatches_.fetch_add(1);
      }
      if (simTime_.totalMilliseconds() != start.t_ms) {
        sim_time_mismatches_.fetch_add(1);
        if (!timeMismatchReported_) {
          RCLCPP_ERROR(this->get_logger(),
                       "SIM_STEP step=%llu published time %llu ms but the coordinator expected %llu ms.",
                       static_cast<unsigned long long>(step),
                       static_cast<unsigned long long>(simTime_.totalMilliseconds()),
                       static_cast<unsigned long long>(start.t_ms));
          timeMismatchReported_ = true;
        }
      }
      if (!completed) {
        RCLCPP_FATAL(get_logger(),
                     "SIM_STEP bridge could not write marker for step=%llu; stopping clock progression.",
                     static_cast<unsigned long long>(step));
        rclcpp::shutdown();
        return false;
      }

      waitForStepClose();
      StepRecord closed_record;
      std::uint64_t timeouts = 0;
      bool closed = false;
      {
        const std::lock_guard<std::mutex> lock(coordinator_mutex_);
        closed_record = stepCoordinator_->record();
        timeouts = stepCoordinator_->totals().timeouts;
        closed = stepCoordinator_->state() == StepState::kClosed;
      }
      if (!closed) {
        return false;  // shutdown while the step was open
      }
      if (closed_record.timed_out) {
        warnStepTimeout(closed_record, timeouts);
      }
      if (replay) {
        replayStepsDone_.fetch_add(1);
      }
      if (logRealTimeFactor_) {
        accountRealTimeFactor(closed_record, step_wall_start, steadyNowNs(), timeouts);
      }
      if (logSimObservability_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "SIM_OBS bridge env_step=%llu sim_time_ms=%llu timeouts=%llu",
          static_cast<unsigned long long>(step),
          static_cast<unsigned long long>(simTime_.totalMilliseconds()),
          static_cast<unsigned long long>(timeouts));
      }
    }
    return true;
  }

  bool AsmSocketCanBridgeNode::runReplay()
  {
    RCLCPP_INFO(this->get_logger(), "SIM_STEP replay started: %zu steps.", replaySteps_.size());
    std::uint64_t previous_ms = 0;
    for (const auto &step : replaySteps_) {
      if (step.t_ms - previous_ms > kMaxStepMs) {
        RCLCPP_ERROR(this->get_logger(),
                     "SIM_STEP replay step %llu advances %llu ms; accepted are 1-%u ms. Replay stopped.",
                     static_cast<unsigned long long>(step.step),
                     static_cast<unsigned long long>(step.t_ms - previous_ms), kMaxStepMs);
        return false;
      }
      previous_ms = step.t_ms;
      if (stop_stepping_.load() || !rclcpp::ok() || !waitForManeuverActive() ||
          !runEnvironmentStep(&step)) {
        return false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "SIM_STEP replay complete: %zu steps, sim_time_ms=%llu.",
                replaySteps_.size(),
                static_cast<unsigned long long>(simTime_.totalMilliseconds()));
    return true;
  }

  void AsmSocketCanBridgeNode::logAppliedCommands(
    std::uint64_t step, std::uint64_t t_ms, const std::vector<SnapshotEntry> &snapshot)
  {
    if (!logSimSteps_) {
      return;
    }
    if (!simCommandLog_.is_open()) {
      std::error_code ignored;
      std::filesystem::create_directories(pathTimeRecord, ignored);
      simCommandLog_.open(std::string(pathTimeRecord) + "/sim_commands.csv");
      if (!simCommandLog_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open %s/sim_commands.csv.", pathTimeRecord.c_str());
        return;
      }
      simCommandLog_ << replayHeader() << '\n';
    }
    simCommandLog_ << formatReplayRow(step, t_ms, snapshot) << '\n';
    simCommandLog_.flush();
  }

  bool AsmSocketCanBridgeNode::runAdaptation()
  {
    const auto window_ms = static_cast<std::uint64_t>(adaptationDurationS_ * 1000.0);
    const auto pace_ns = static_cast<std::int64_t>(1e6 / adaptationRealtimeFactor_);
    RCLCPP_INFO(this->get_logger(),
                "SIM_STEP adaptation started: window=%llu ms, 1 ms steps paced at realtime_factor %.3f.",
                static_cast<unsigned long long>(window_ms), adaptationRealtimeFactor_);

    std::optional<DerivedSchedule> derived;
    std::uint64_t switch_ms = 0;
    while (!stop_stepping_.load() && rclcpp::ok()) {
      if (!waitForManeuverActive()) {
        return false;
      }
      const auto step_wall_start = steadyNowNs();
      if (!runEnvironmentStep()) {
        return false;
      }
      const auto t_ms = simTime_.totalMilliseconds();
      if (!derived && t_ms >= window_ms) {
        const std::lock_guard<std::mutex> lock(coordinator_mutex_);
        derived = adaptationDetector_->derive(enabledOutputIntervalsMs_, t_ms + 1);
        // Switch at the first multiple of the derived step so that the stack only sees larger increments.
        switch_ms = (t_ms + derived->step_ms - 1) / derived->step_ms * derived->step_ms;
      }
      if (derived && t_ms >= switch_ms) {
        break;
      }
      std::unique_lock<std::mutex> lock(coordinator_mutex_);
      const std::chrono::steady_clock::time_point deadline{
        std::chrono::nanoseconds(step_wall_start + pace_ns)};
      coordinator_cv_.wait_until(lock, deadline, [this]() {return stop_stepping_.load();});
    }
    if (!derived) {
      return false;
    }
    finishAdaptation(*derived, switch_ms);
    return true;
  }

  void AsmSocketCanBridgeNode::finishAdaptation(const DerivedSchedule &derived, std::uint64_t switch_ms)
  {
    DerivedSchedule final_schedule = derived;
    final_schedule.schedule.clear();
    for (auto &id : final_schedule.ids) {
      if (id.scheduled) {
        id.first_due_ms = AdaptationDetector::firstGridTimeAtOrAfter(
          id.anchor_ms, id.period_ms, switch_ms + 1);
        final_schedule.schedule.push_back({id.id, id.period_ms, id.first_due_ms});
      }
    }

    StepCoordinator::Config config;
    config.step_ms = final_schedule.step_ms;
    config.timeout_ns = activeTimeoutMs_ * 1000000;
    config.schedule = final_schedule.schedule;
    {
      const std::lock_guard<std::mutex> lock(coordinator_mutex_);
      adaptationDetector_.reset();
      stepCoordinator_->reconfigure(std::move(config));
    }
    sim_step_ms_ = final_schedule.step_ms;

    std::size_t scheduled = 0;
    for (const auto &id : final_schedule.ids) {
      scheduled += id.scheduled ? 1 : 0;
      RCLCPP_INFO(this->get_logger(),
                  "SIM_STEP adaptation id=0x%X arrivals=%u class=%s period_ms=%u anchor_ms=%u "
                  "first_due_ms=%u jitter_ms=%u missing_cycles=%u latency_us p50=%lld p99=%lld max=%lld",
                  id.id, id.arrivals,
                  id.scheduled ? "scheduled" : (id.wall_timer_suspect ? "unscheduled-wall-timer" : "unscheduled"),
                  id.period_ms, id.anchor_ms, id.first_due_ms, id.jitter_ms, id.missing_cycles,
                  static_cast<long long>(id.latency_p50_ns / 1000),
                  static_cast<long long>(id.latency_p99_ns / 1000),
                  static_cast<long long>(id.latency_max_ns / 1000));
    }
    RCLCPP_INFO(this->get_logger(),
                "SIM_STEP adaptation complete: step=%u ms timeout=%lld ms switch_ms=%llu window_ms=%llu "
                "scheduled=%zu unscheduled=%zu",
                final_schedule.step_ms, static_cast<long long>(activeTimeoutMs_),
                static_cast<unsigned long long>(switch_ms),
                static_cast<unsigned long long>(adaptationDurationS_ * 1000.0),
                scheduled, final_schedule.ids.size() - scheduled);
    writeAdaptationReport(final_schedule, switch_ms);
  }

  void AsmSocketCanBridgeNode::writeAdaptationReport(const DerivedSchedule &derived, std::uint64_t switch_ms)
  {
    std::error_code ignored;
    std::filesystem::create_directories(pathTimeRecord, ignored);
    std::ofstream report(std::string(pathTimeRecord) + "/adaptation.txt");
    if (!report.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not write %s/adaptation.txt.", pathTimeRecord.c_str());
      return;
    }
    report << "# step_ms=" << derived.step_ms << " timeout_ms=" << activeTimeoutMs_
           << " switch_ms=" << switch_ms << " window_ms=" << adaptationDurationS_ * 1000.0 << '\n'
           << "id,arrivals,class,period_ms,anchor_ms,first_due_ms,jitter_ms,missing_cycles,"
              "latency_p50_us,latency_p95_us,latency_p99_us,latency_max_us\n";
    for (const auto &id : derived.ids) {
      report << id.id << ',' << id.arrivals << ','
             << (id.scheduled ? "scheduled" : (id.wall_timer_suspect ? "unscheduled-wall-timer" : "unscheduled"))
             << ',' << id.period_ms << ',' << id.anchor_ms << ',' << id.first_due_ms << ','
             << id.jitter_ms << ',' << id.missing_cycles << ',' << id.latency_p50_ns / 1000 << ','
             << id.latency_p95_ns / 1000 << ',' << id.latency_p99_ns / 1000 << ','
             << id.latency_max_ns / 1000 << '\n';
    }
  }

  void AsmSocketCanBridgeNode::accountRealTimeFactor(
    const StepRecord &record, std::int64_t step_start_ns, std::int64_t step_end_ns,
    std::uint64_t timeouts)
  {
    const std::int64_t wait_ns = record.close_ns - record.release_ns;
    const std::int64_t env_ns = (step_end_ns - step_start_ns) - wait_ns;
    rtfTotal_.sim_ms += sim_step_ms_;
    rtfTotal_.env_ns += env_ns;
    rtfTotal_.wait_ns += wait_ns;
    if (simTime_.totalMilliseconds() < rtfNextLogMs_) {
      return;
    }
    const double wall_ms = static_cast<double>(rtfTotal_.env_ns + rtfTotal_.wait_ns) / 1e6;
    const double realtime_factor =
      wall_ms > 0.0 ? static_cast<double>(rtfTotal_.sim_ms) / wall_ms : 0.0;
    RCLCPP_INFO(this->get_logger(),
                "SIM_REALTIME_FACTOR sim_time_ms=%llu realtime_factor=%.3f timeouts=%llu",
                static_cast<unsigned long long>(simTime_.totalMilliseconds()), realtime_factor,
                static_cast<unsigned long long>(timeouts));
    while (rtfNextLogMs_ <= simTime_.totalMilliseconds()) {
      rtfNextLogMs_ += kRealtimeFactorLogIntervalMs;
    }
  }

  void AsmSocketCanBridgeNode::warnStepTimeout(const StepRecord &record, std::uint64_t total_timeouts)
  {
    const auto now = steadyNowNs();
    if (lastTimeoutWarnNs_ != 0 && now - lastTimeoutWarnNs_ < warning_throttle_intervall * 1000000) {
      ++suppressedTimeoutWarnings_;
      return;
    }
    std::string missing;
    for (const auto id : record.missing_ids) {
      missing += (missing.empty() ? "0x" : ",0x");
      char buffer[16];
      std::snprintf(buffer, sizeof(buffer), "%X", id);
      missing += buffer;
    }
    RCLCPP_WARN(this->get_logger(),
                "SIM_STEP step=%llu timed out after %lld ms; missing command IDs: %s; "
                "last values held; %llu timeouts in total; %llu earlier timeout warnings suppressed",
                static_cast<unsigned long long>(record.step),
                static_cast<long long>(activeTimeoutMs_), missing.c_str(),
                static_cast<unsigned long long>(total_timeouts),
                static_cast<unsigned long long>(suppressedTimeoutWarnings_));
    lastTimeoutWarnNs_ = now;
    suppressedTimeoutWarnings_ = 0;
  }

  void AsmSocketCanBridgeNode::logStepRecord(const StepRecord &record)
  {
    if (!logSimSteps_) {
      return;
    }
    if (!simStepLog_.is_open()) {
      std::error_code ignored;
      std::filesystem::create_directories(pathTimeRecord, ignored);
      simStepLog_.open(std::string(pathTimeRecord) + "/sim_steps.csv");
      if (!simStepLog_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open %s/sim_steps.csv; step logging disabled.",
                     pathTimeRecord.c_str());
        logSimSteps_ = false;
        return;
      }
      simStepLog_ << "step,t_ms,release_ns,close_ns,wait_us,timed_out,due,missing,accepted,"
                     "duplicates,late,early,arrivals_us\n";
    }
    const auto join_ids = [](const std::vector<std::uint32_t> &ids) {
      std::string text;
      for (const auto id : ids) {
        text += (text.empty() ? "" : "|") + std::to_string(id);
      }
      return text;
    };
    std::string arrivals;
    for (const auto &arrival : record.arrivals) {
      arrivals += (arrivals.empty() ? "" : "|") + std::to_string(arrival.id) + ":" +
        std::to_string(arrival.latency_ns / 1000);
    }
    simStepLog_ << record.step << ',' << record.t_ms << ',' << record.release_ns << ','
                << record.close_ns << ',' << (record.close_ns - record.release_ns) / 1000 << ','
                << (record.timed_out ? 1 : 0) << ',' << join_ids(record.due_ids) << ','
                << join_ids(record.missing_ids) << ',' << record.accepted << ','
                << record.duplicates << ',' << record.late << ',' << record.early << ','
                << arrivals << '\n';
    simStepLog_.flush();
  }

} // namespace asm_socketcan_bridge
