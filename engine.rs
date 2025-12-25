#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]
#![deny(clippy::float_cmp)]

// ===================== CORE CONSTANTS =====================
pub const MAX_TEMP_DECICELSIUS: i16 = 850;
pub const OVERHEAT_THRESHOLD_DECICELSIUS: i16 = 750;
pub const NORMAL_THRESHOLD_DECICELSIUS: i16 = 700;
pub const COOLDOWN_THRESHOLD_DECICELSIUS: i16 = 650;
pub const SPIKE_THRESHOLD_DECICELSIUS: i16 = 800;
pub const FULLY_COOLED_THRESHOLD_DECICELSIUS: i16 = 200;

pub const INFERENCE_DEADLINE_MICROS: u64 = 20_000;
pub const WATCHDOG_TIMEOUT_MICROS: u64 = 1_000_000;
pub const TELEMETRY_INTERVAL_MICROS: u64 = 5_000_000;

// ===================== LOGGER ABSTRACTION =====================
pub trait Logger {
    fn log(&self, level: LogLevel, message: &str);
    fn log_telemetry(&self, snapshot: TelemetrySnapshot, state: RuntimeState, temp: i16, timestamp: u64, inference_count: u32);
}

#[derive(Clone, Copy)]
pub enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

pub struct NoopLogger;

impl Logger for NoopLogger {
    fn log(&self, _level: LogLevel, _message: &str) {}
    fn log_telemetry(&self, _snapshot: TelemetrySnapshot, _state: RuntimeState, _temp: i16, _timestamp: u64, _inference_count: u32) {}
}

#[cfg(feature = "embassy")]
pub struct EmbassyLogger;

#[cfg(feature = "embassy")]
impl Logger for EmbassyLogger {
    fn log(&self, level: LogLevel, message: &str) {
        use cortex_m_semihosting::hprintln;
        let _ = hprintln!("[{:?}] {}", level, message);
    }
    
    fn log_telemetry(&self, snapshot: TelemetrySnapshot, state: RuntimeState, temp: i16, timestamp: u64, inference_count: u32) {
        use cortex_m_semihosting::hprintln;
        let _ = hprintln!(
            "TELEMETRY[{}]: state={:?} temp={}.{}°C inferences={} latency={}μs miss_rate={}.{}%",
            timestamp,
            state,
            temp / 10,
            temp % 10,
            inference_count,
            snapshot.avg_latency_us,
            snapshot.deadline_miss_rate_perthousand / 10,
            snapshot.deadline_miss_rate_perthousand % 10
        );
    }
}

// ===================== CLOCK ABSTRACTION =====================
pub trait Clock {
    fn now_micros(&self) -> u64;
    fn elapsed_since(&self, start_micros: u64) -> u64;
}

pub struct MonotonicClock<C: Clock> {
    inner: C,
    base: u64,
}

impl<C: Clock> MonotonicClock<C> {
    pub fn new(inner: C) -> Self {
        let base = inner.now_micros();
        Self { inner, base }
    }
    
    pub fn monotonic(&self) -> u64 {
        self.inner.now_micros().wrapping_sub(self.base)
    }
}

#[cfg(feature = "embassy")]
pub struct EmbassyClock;

#[cfg(feature = "embassy")]
impl Clock for EmbassyClock {
    fn now_micros(&self) -> u64 {
        embassy_time::Instant::now().as_micros() as u64
    }
    
    fn elapsed_since(&self, start_micros: u64) -> u64 {
        let current = self.now_micros();
        current.saturating_sub(start_micros)
    }
}

// ===================== WATCHDOG ABSTRACTION =====================
pub trait Watchdog<C: Clock> {
    fn feed(&mut self);
    fn is_expired(&self) -> bool;
    fn reset(&mut self);
    fn disable(&mut self);
}

pub struct HardwareWatchdog<C: Clock> {
    clock: MonotonicClock<C>,
    last_feed: u64,
    timeout_micros: u64,
    enabled: bool,
}

impl<C: Clock> HardwareWatchdog<C> {
    pub fn new(clock: MonotonicClock<C>, timeout_micros: u64) -> Self {
        let now = clock.monotonic();
        Self {
            clock,
            last_feed: now,
            timeout_micros,
            enabled: true,
        }
    }
}

impl<C: Clock> Watchdog<C> for HardwareWatchdog<C> {
    fn feed(&mut self) {
        self.last_feed = self.clock.monotonic();
    }
    
    fn is_expired(&self) -> bool {
        if !self.enabled {
            return false;
        }
        let now = self.clock.monotonic();
        now.saturating_sub(self.last_feed) >= self.timeout_micros
    }
    
    fn reset(&mut self) {
        self.last_feed = self.clock.monotonic();
        self.enabled = true;
    }
    
    fn disable(&mut self) {
        self.enabled = false;
    }
}

// ===================== SAFETY POLICY =====================
pub mod safety {
    use super::*;
    
    #[derive(Clone, Copy)]
    pub enum SafetyCondition {
        Temperature { max: i16 },
        Rate { max_perthousand: u32 },
        Boolean,
    }
    
    impl SafetyCondition {
        pub const fn check(&self, value: SafetyValue) -> bool {
            match (self, value) {
                (SafetyCondition::Temperature { max }, SafetyValue::Temperature(temp)) => temp < *max,
                (SafetyCondition::Rate { max_perthousand }, SafetyValue::Rate(rate)) => rate < *max_perthousand,
                (SafetyCondition::Boolean, SafetyValue::Boolean(value)) => value,
                _ => false,
            }
        }
    }
    
    pub enum SafetyValue {
        Temperature(i16),
        Rate(u32),
        Boolean(bool),
    }
    
    #[derive(Clone, Copy)]
    pub struct SafetyRequirement<const N: usize> {
        pub id: u32,
        pub description: &'static str,
        pub condition: SafetyCondition,
        pub max_faults: u32,
        pub fault_action: FaultAction,
    }
    
    #[derive(Clone, Copy)]
    pub enum FaultAction {
        Warning,
        ReduceLoad,
        EmergencyStop,
        Shutdown,
    }
    
    pub struct SafetyMonitor<const N: usize> {
        requirements: [SafetyRequirement<N>; N],
        fault_counters: [u32; N],
        total_faults: u32,
    }
    
    impl<const N: usize> SafetyMonitor<N> {
        pub const fn new(requirements: [SafetyRequirement<N>; N]) -> Self {
            Self {
                requirements,
                fault_counters: [0; N],
                total_faults: 0,
            }
        }
        
        pub fn check(&mut self, id: usize, value: SafetyValue) -> Result<(), SafetyViolation> {
            if id >= N {
                return Err(SafetyViolation::InvalidRequirement);
            }
            
            let req = &self.requirements[id];
            let passed = req.condition.check(value);
            
            if !passed {
                self.fault_counters[id] = self.fault_counters[id].saturating_add(1);
                self.total_faults = self.total_faults.saturating_add(1);
                
                if self.fault_counters[id] >= req.max_faults {
                    return Err(SafetyViolation::Critical {
                        requirement_id: req.id,
                        action: req.fault_action,
                    });
                } else {
                    return Err(SafetyViolation::Warning(req.id));
                }
            }
            
            Ok(())
        }
        
        pub const fn get_fault_count(&self, id: usize) -> u32 {
            if id < N {
                self.fault_counters[id]
            } else {
                0
            }
        }
        
        pub const fn total_faults(&self) -> u32 {
            self.total_faults
        }
        
        pub fn reset(&mut self) {
            for counter in &mut self.fault_counters {
                *counter = 0;
            }
            self.total_faults = 0;
        }
    }
    
    #[derive(Clone, Copy)]
    pub enum SafetyViolation {
        InvalidRequirement,
        Warning(u32),
        Critical {
            requirement_id: u32,
            action: FaultAction,
        },
    }
}

// ===================== TYPESTATE FSM =====================
pub mod typestate {
    use super::*;
    
    pub trait State {
        const CAN_INFER: bool;
        const IS_SAFE: bool;
        fn runtime_enum() -> RuntimeState;
    }
    
    pub struct Initializing;
    pub struct Normal;
    pub struct Overheating;
    pub struct EmergencyStop;
    pub struct CoolingDown;
    pub struct Shutdown;
    
    impl State for Initializing {
        const CAN_INFER: bool = false;
        const IS_SAFE: bool = false;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::Initializing
        }
    }
    
    impl State for Normal {
        const CAN_INFER: bool = true;
        const IS_SAFE: bool = true;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::Normal
        }
    }
    
    impl State for Overheating {
        const CAN_INFER: bool = true;
        const IS_SAFE: bool = true;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::Overheating
        }
    }
    
    impl State for EmergencyStop {
        const CAN_INFER: bool = false;
        const IS_SAFE: bool = false;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::EmergencyStop
        }
    }
    
    impl State for CoolingDown {
        const CAN_INFER: bool = false;
        const IS_SAFE: bool = false;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::CoolingDown
        }
    }
    
    impl State for Shutdown {
        const CAN_INFER: bool = false;
        const IS_SAFE: bool = false;
        fn runtime_enum() -> RuntimeState {
            RuntimeState::Shutdown
        }
    }
    
    pub struct System<S: State> {
        temperature: i16,
        timestamp: u64,
        _marker: core::marker::PhantomData<S>,
    }
    
    impl System<Initializing> {
        pub fn new() -> Self {
            Self {
                temperature: 250,
                timestamp: 0,
                _marker: core::marker::PhantomData,
            }
        }
        
        pub fn init_complete(self) -> Result<System<Normal>, TransitionError> {
            if self.temperature < OVERHEAT_THRESHOLD_DECICELSIUS {
                Ok(System {
                    temperature: self.temperature,
                    timestamp: self.timestamp,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::TemperatureTooHigh)
            }
        }
    }
    
    impl System<Normal> {
        pub fn temperature_high(self, temp: i16) -> Result<System<Overheating>, TransitionError> {
            if temp >= OVERHEAT_THRESHOLD_DECICELSIUS && temp < MAX_TEMP_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn temperature_critical(self, temp: i16) -> Result<System<EmergencyStop>, TransitionError> {
            if temp >= MAX_TEMP_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn shutdown(self) -> System<Shutdown> {
            System {
                temperature: self.temperature,
                timestamp: self.timestamp,
                _marker: core::marker::PhantomData,
            }
        }
    }
    
    impl System<Overheating> {
        pub fn temperature_normal(self, temp: i16) -> Result<System<Normal>, TransitionError> {
            if temp < NORMAL_THRESHOLD_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn temperature_critical(self, temp: i16) -> Result<System<EmergencyStop>, TransitionError> {
            if temp >= MAX_TEMP_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn shutdown(self) -> System<Shutdown> {
            System {
                temperature: self.temperature,
                timestamp: self.timestamp,
                _marker: core::marker::PhantomData,
            }
        }
    }
    
    impl System<EmergencyStop> {
        pub fn cooldown_complete(self, temp: i16) -> Result<System<CoolingDown>, TransitionError> {
            if temp <= COOLDOWN_THRESHOLD_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
    }
    
    impl System<CoolingDown> {
        pub fn fully_cooled(self, temp: i16) -> Result<System<Normal>, TransitionError> {
            if temp <= FULLY_COOLED_THRESHOLD_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn temperature_spike(self, temp: i16) -> Result<System<EmergencyStop>, TransitionError> {
            if temp >= SPIKE_THRESHOLD_DECICELSIUS {
                Ok(System {
                    temperature: temp,
                    timestamp: self.timestamp + 1,
                    _marker: core::marker::PhantomData,
                })
            } else {
                Err(TransitionError::InvalidTemperature)
            }
        }
        
        pub fn shutdown(self) -> System<Shutdown> {
            System {
                temperature: self.temperature,
                timestamp: self.timestamp,
                _marker: core::marker::PhantomData,
            }
        }
    }
    
    impl<S: State> System<S> {
        pub const CAN_INFER: bool = S::CAN_INFER;
        pub const IS_SAFE: bool = S::IS_SAFE;
        
        pub fn temperature(&self) -> i16 {
            self.temperature
        }
        
        pub fn runtime_state(&self) -> RuntimeState {
            S::runtime_enum()
        }
    }
    
    #[derive(Clone, Copy)]
    pub enum TransitionError {
        TemperatureTooHigh,
        InvalidTemperature,
        InvalidState,
    }
}

// ===================== UNIFIED FSM =====================
#[derive(Clone, Copy)]
pub enum RuntimeState {
    Initializing = 0,
    Normal = 1,
    Overheating = 2,
    EmergencyStop = 3,
    CoolingDown = 4,
    Shutdown = 5,
}

pub enum StateMachine {
    Initializing(typestate::System<typestate::Initializing>),
    Normal(typestate::System<typestate::Normal>),
    Overheating(typestate::System<typestate::Overheating>),
    EmergencyStop(typestate::System<typestate::EmergencyStop>),
    CoolingDown(typestate::System<typestate::CoolingDown>),
    Shutdown(typestate::System<typestate::Shutdown>),
}

impl StateMachine {
    pub fn new() -> Self {
        StateMachine::Initializing(typestate::System::new())
    }
    
    pub fn runtime_state(&self) -> RuntimeState {
        match self {
            StateMachine::Initializing(_) => RuntimeState::Initializing,
            StateMachine::Normal(_) => RuntimeState::Normal,
            StateMachine::Overheating(_) => RuntimeState::Overheating,
            StateMachine::EmergencyStop(_) => RuntimeState::EmergencyStop,
            StateMachine::CoolingDown(_) => RuntimeState::CoolingDown,
            StateMachine::Shutdown(_) => RuntimeState::Shutdown,
        }
    }
    
    pub fn can_do_inference(&self) -> bool {
        match self {
            StateMachine::Normal(s) => s.CAN_INFER,
            StateMachine::Overheating(s) => s.CAN_INFER,
            _ => false,
        }
    }
    
    pub fn is_safe(&self) -> bool {
        match self {
            StateMachine::Initializing(s) => s.IS_SAFE,
            StateMachine::Normal(s) => s.IS_SAFE,
            StateMachine::Overheating(s) => s.IS_SAFE,
            StateMachine::EmergencyStop(s) => s.IS_SAFE,
            StateMachine::CoolingDown(s) => s.IS_SAFE,
            StateMachine::Shutdown(s) => s.IS_SAFE,
        }
    }
    
    pub fn temperature(&self) -> i16 {
        match self {
            StateMachine::Initializing(s) => s.temperature(),
            StateMachine::Normal(s) => s.temperature(),
            StateMachine::Overheating(s) => s.temperature(),
            StateMachine::EmergencyStop(s) => s.temperature(),
            StateMachine::CoolingDown(s) => s.temperature(),
            StateMachine::Shutdown(s) => s.temperature(),
        }
    }
    
    pub fn handle_safety_violation(self, action: safety::FaultAction, temp: i16) -> Result<Self, typestate::TransitionError> {
        match action {
            safety::FaultAction::EmergencyStop => self.temperature_critical(temp),
            safety::FaultAction::Shutdown => self.shutdown(),
            _ => Ok(self),
        }
    }
    
    pub fn init_complete(self) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::Initializing(s) => {
                let new_state = s.init_complete()?;
                Ok(StateMachine::Normal(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn temperature_high(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::Normal(s) => {
                let new_state = s.temperature_high(temp)?;
                Ok(StateMachine::Overheating(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn temperature_normal(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::Overheating(s) => {
                let new_state = s.temperature_normal(temp)?;
                Ok(StateMachine::Normal(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn temperature_critical(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::Normal(s) => {
                let new_state = s.temperature_critical(temp)?;
                Ok(StateMachine::EmergencyStop(new_state))
            }
            StateMachine::Overheating(s) => {
                let new_state = s.temperature_critical(temp)?;
                Ok(StateMachine::EmergencyStop(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn cooldown_complete(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::EmergencyStop(s) => {
                let new_state = s.cooldown_complete(temp)?;
                Ok(StateMachine::CoolingDown(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn fully_cooled(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::CoolingDown(s) => {
                let new_state = s.fully_cooled(temp)?;
                Ok(StateMachine::Normal(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn temperature_spike(self, temp: i16) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::CoolingDown(s) => {
                let new_state = s.temperature_spike(temp)?;
                Ok(StateMachine::EmergencyStop(new_state))
            }
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
    
    pub fn shutdown(self) -> Result<Self, typestate::TransitionError> {
        match self {
            StateMachine::Normal(s) => Ok(StateMachine::Shutdown(s.shutdown())),
            StateMachine::Overheating(s) => Ok(StateMachine::Shutdown(s.shutdown())),
            StateMachine::CoolingDown(s) => Ok(StateMachine::Shutdown(s.shutdown())),
            _ => Err(typestate::TransitionError::InvalidState),
        }
    }
}

// ===================== MAIN SYSTEM =====================
pub struct AiotSystem<C: Clock, W: Watchdog<C>, L: Logger, const N: usize> {
    state_machine: StateMachine,
    telemetry: TelemetryRingBuffer<1000>,
    safety_monitor: safety::SafetyMonitor<N>,
    inference_count: u32,
    watchdog: W,
    last_telemetry_report: u64,
    clock: C,
    logger: L,
}

impl<C: Clock, W: Watchdog<C>, L: Logger, const N: usize> AiotSystem<C, W, L, N> {
    pub fn new(
        safety_requirements: [safety::SafetyRequirement<N>; N],
        clock: C,
        watchdog: W,
        logger: L,
    ) -> Self {
        Self {
            state_machine: StateMachine::new(),
            telemetry: TelemetryRingBuffer::new(),
            safety_monitor: safety::SafetyMonitor::new(safety_requirements),
            inference_count: 0,
            watchdog,
            last_telemetry_report: 0,
            clock,
            logger,
        }
    }
    
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        self.watchdog.reset();
        let new_state = self.state_machine.init_complete()
            .map_err(SystemError::StateTransitionError)?;
        self.state_machine = new_state;
        Ok(())
    }
    
    pub fn process_temperature(&mut self, temp_decicelsius: i16) -> Result<(), SystemError> {
        if self.watchdog.is_expired() {
            return Err(SystemError::WatchdogTimeout);
        }
        
        let check_result = self.safety_monitor.check(
            0,
            safety::SafetyValue::Temperature(temp_decicelsius),
        );
        
        match check_result {
            Ok(_) => {
                self.update_state_based_on_temperature(temp_decicelsius)?;
            }
            Err(safety::SafetyViolation::Critical { action, .. }) => {
                self.state_machine = self.state_machine.handle_safety_violation(action, temp_decicelsius)
                    .map_err(SystemError::StateTransitionError)?;
            }
            Err(safety::SafetyViolation::Warning(id)) => {
                self.logger.log(LogLevel::Warn, &format!("Safety warning: requirement {}", id));
                self.update_state_based_on_temperature(temp_decicelsius)?;
            }
            Err(safety::SafetyViolation::InvalidRequirement) => {
                return Err(SystemError::SafetyViolation);
            }
        }
        
        self.watchdog.feed();
        Ok(())
    }
    
    fn update_state_based_on_temperature(&mut self, temp: i16) -> Result<(), SystemError> {
        let result = match self.state_machine.runtime_state() {
            RuntimeState::Normal => {
                if temp >= MAX_TEMP_DECICELSIUS {
                    self.state_machine.temperature_critical(temp)
                } else if temp >= OVERHEAT_THRESHOLD_DECICELSIUS {
                    self.state_machine.temperature_high(temp)
                } else {
                    Ok(self.state_machine)
                }
            }
            RuntimeState::Overheating => {
                if temp >= MAX_TEMP_DECICELSIUS {
                    self.state_machine.temperature_critical(temp)
                } else if temp < NORMAL_THRESHOLD_DECICELSIUS {
                    self.state_machine.temperature_normal(temp)
                } else {
                    Ok(self.state_machine)
                }
            }
            RuntimeState::EmergencyStop => {
                if temp <= COOLDOWN_THRESHOLD_DECICELSIUS {
                    self.state_machine.cooldown_complete(temp)
                } else {
                    Ok(self.state_machine)
                }
            }
            RuntimeState::CoolingDown => {
                if temp <= FULLY_COOLED_THRESHOLD_DECICELSIUS {
                    self.state_machine.fully_cooled(temp)
                } else if temp >= SPIKE_THRESHOLD_DECICELSIUS {
                    self.state_machine.temperature_spike(temp)
                } else {
                    Ok(self.state_machine)
                }
            }
            _ => Ok(self.state_machine),
        };
        
        self.state_machine = result.map_err(SystemError::StateTransitionError)?;
        Ok(())
    }
    
    pub fn run_inference(&mut self, prediction: [i16; 3]) -> Result<[i16; 3], SystemError> {
        if self.watchdog.is_expired() {
            return Err(SystemError::WatchdogTimeout);
        }
        
        if !self.state_machine.can_do_inference() {
            return Err(SystemError::InferenceNotAllowed);
        }
        
        let checksum_valid = true;
        let checksum_result = self.safety_monitor.check(
            2,
            safety::SafetyValue::Boolean(checksum_valid),
        );
        
        if let Err(safety::SafetyViolation::Critical { action, .. }) = checksum_result {
            match action {
                safety::FaultAction::Shutdown => {
                    self.state_machine = self.state_machine.shutdown()
                        .map_err(SystemError::StateTransitionError)?;
                    self.watchdog.disable();
                    return Err(SystemError::SafetyShutdown);
                }
                _ => {}
            }
        }
        
        if !checksum_valid {
            return Err(SystemError::ModelError(ModelError::ChecksumMismatch));
        }
        
        let start_time = self.clock.now_micros();
        let latency = self.clock.elapsed_since(start_time);
        let power_mw = 5000u16;
        let deadline_missed = latency > INFERENCE_DEADLINE_MICROS;
        
        self.telemetry.add_sample(
            latency as u32,
            power_mw,
            deadline_missed
        );
        
        let snapshot = self.telemetry.snapshot();
        let _ = self.safety_monitor.check(
            1,
            safety::SafetyValue::Rate(snapshot.deadline_miss_rate_perthousand),
        );
        
        self.inference_count = self.inference_count.wrapping_add(1);
        self.watchdog.feed();
        
        let now = self.clock.now_micros();
        if now - self.last_telemetry_report > TELEMETRY_INTERVAL_MICROS {
            self.logger.log_telemetry(
                snapshot,
                self.state_machine.runtime_state(),
                self.state_machine.temperature(),
                now,
                self.inference_count,
            );
            self.last_telemetry_report = now;
        }
        
        Ok(prediction)
    }
    
    pub fn force_shutdown(&mut self) -> Result<(), SystemError> {
        self.state_machine = self.state_machine.shutdown()
            .map_err(SystemError::StateTransitionError)?;
        self.watchdog.disable();
        Ok(())
    }
    
    pub fn telemetry_snapshot(&self) -> TelemetrySnapshot {
        self.telemetry.snapshot()
    }
    
    pub const fn safety_fault_counts(&self) -> [u32; N] {
        let mut counts = [0; N];
        let mut i = 0;
        while i < N {
            counts[i] = self.safety_monitor.get_fault_count(i);
            i += 1;
        }
        counts
    }
    
    pub const fn total_safety_faults(&self) -> u32 {
        self.safety_monitor.total_faults()
    }
}

// ===================== SUPPORTING STRUCTS =====================
pub struct TelemetryRingBuffer<const N: usize> {
    latencies: [u32; N],
    powers: [u16; N],
    deadlines_missed: [bool; N],
    read_index: usize,
    write_index: usize,
    count: usize,
    total_samples: u64,
}

impl<const N: usize> TelemetryRingBuffer<N> {
    pub const fn new() -> Self {
        Self {
            latencies: [0; N],
            powers: [0; N],
            deadlines_missed: [false; N],
            read_index: 0,
            write_index: 0,
            count: 0,
            total_samples: 0,
        }
    }
    
    pub fn add_sample(&mut self, latency_us: u32, power_mw: u16, deadline_missed: bool) {
        self.latencies[self.write_index] = latency_us;
        self.powers[self.write_index] = power_mw;
        self.deadlines_missed[self.write_index] = deadline_missed;
        
        if self.count < N {
            self.count += 1;
        } else {
            self.read_index = (self.read_index + 1) % N;
        }
        
        self.write_index = (self.write_index + 1) % N;
        self.total_samples = self.total_samples.saturating_add(1);
    }
    
    pub fn snapshot(&self) -> TelemetrySnapshot {
        if self.count == 0 {
            return TelemetrySnapshot {
                total_samples: 0,
                deadline_miss_rate_perthousand: 0,
                avg_latency_us: 0,
                avg_power_mw: 0,
                min_latency_us: 0,
                max_latency_us: 0,
                deadline_misses: 0,
            };
        }
        
        let mut total_latency = 0u64;
        let mut total_power = 0u64;
        let mut missed_count = 0u64;
        let mut min_latency = u32::MAX;
        let mut max_latency = 0u32;
        
        let mut idx = self.read_index;
        for _ in 0..self.count {
            total_latency += self.latencies[idx] as u64;
            total_power += self.powers[idx] as u64;
            
            if self.deadlines_missed[idx] {
                missed_count += 1;
            }
            
            if self.latencies[idx] < min_latency {
                min_latency = self.latencies[idx];
            }
            
            if self.latencies[idx] > max_latency {
                max_latency = self.latencies[idx];
            }
            
            idx = (idx + 1) % N;
        }
        
        TelemetrySnapshot {
            total_samples: self.total_samples as u32,
            deadline_miss_rate_perthousand: (missed_count * 1000 / self.count as u64) as u32,
            avg_latency_us: (total_latency / self.count as u64) as u32,
            avg_power_mw: (total_power / self.count as u64) as u16,
            min_latency_us: min_latency,
            max_latency_us: max_latency,
            deadline_misses: missed_count as u32,
        }
    }
}

#[derive(Clone, Copy)]
pub struct TelemetrySnapshot {
    pub total_samples: u32,
    pub deadline_miss_rate_perthousand: u32,
    pub avg_latency_us: u32,
    pub avg_power_mw: u16,
    pub min_latency_us: u32,
    pub max_latency_us: u32,
    pub deadline_misses: u32,
}

#[derive(Clone, Copy)]
pub enum ModelError {
    ChecksumMismatch,
    InvalidQuantization,
    InferenceError,
}

#[derive(Clone, Copy)]
pub enum SystemError {
    ModelError(ModelError),
    StateTransitionError(typestate::TransitionError),
    InferenceNotAllowed,
    WatchdogTimeout,
    SafetyShutdown,
    SafetyViolation,
}

// ===================== TESTS =====================
#[cfg(feature = "test")]
pub mod test {
    use super::*;
    
    #[derive(Clone)]
    pub struct TestClock {
        current_time: u64,
        monotonic_time: u64,
    }
    
    impl TestClock {
        pub fn new() -> Self {
            Self {
                current_time: 0,
                monotonic_time: 0,
            }
        }
        
        pub fn advance(&mut self, microseconds: u64) {
            self.current_time = self.current_time.wrapping_add(microseconds);
            self.monotonic_time = self.monotonic_time.wrapping_add(microseconds);
        }
        
        pub fn set_time(&mut self, microseconds: u64) {
            self.current_time = microseconds;
        }
        
        pub fn set_monotonic(&mut self, microseconds: u64) {
            self.monotonic_time = microseconds;
        }
    }
    
    impl Clock for TestClock {
        fn now_micros(&self) -> u64 {
            self.current_time
        }
        
        fn elapsed_since(&self, start_micros: u64) -> u64 {
            self.current_time.saturating_sub(start_micros)
        }
    }
    
    pub struct TestWatchdog<C: Clock> {
        clock: MonotonicClock<C>,
        last_feed: u64,
        timeout_micros: u64,
        enabled: bool,
        expire_at: Option<u64>,
    }
    
    impl<C: Clock> TestWatchdog<C> {
        pub fn new(clock: MonotonicClock<C>, timeout_micros: u64) -> Self {
            let now = clock.monotonic();
            Self {
                clock,
                last_feed: now,
                timeout_micros,
                enabled: true,
                expire_at: None,
            }
        }
        
        pub fn trigger_expiration(&mut self, at_monotonic: u64) {
            self.expire_at = Some(at_monotonic);
        }
    }
    
    impl<C: Clock> Watchdog<C> for TestWatchdog<C> {
        fn feed(&mut self) {
            self.last_feed = self.clock.monotonic();
            self.expire_at = None;
        }
        
        fn is_expired(&self) -> bool {
            if !self.enabled {
                return false;
            }
            
            if let Some(expire_at) = self.expire_at {
                if self.clock.monotonic() >= expire_at {
                    return true;
                }
            }
            
            let now = self.clock.monotonic();
            now.saturating_sub(self.last_feed) >= self.timeout_micros
        }
        
        fn reset(&mut self) {
            self.last_feed = self.clock.monotonic();
            self.enabled = true;
            self.expire_at = None;
        }
        
        fn disable(&mut self) {
            self.enabled = false;
        }
    }
    
    pub struct TestLogger {
        logs: Vec<(LogLevel, String)>,
        telemetry_logs: Vec<(TelemetrySnapshot, RuntimeState, i16, u64, u32)>,
    }
    
    impl TestLogger {
        pub fn new() -> Self {
            Self {
                logs: Vec::new(),
                telemetry_logs: Vec::new(),
            }
        }
        
        pub fn get_logs(&self) -> &[(LogLevel, String)] {
            &self.logs
        }
        
        pub fn get_telemetry_logs(&self) -> &[(TelemetrySnapshot, RuntimeState, i16, u64, u32)] {
            &self.telemetry_logs
        }
        
        pub fn clear(&mut self) {
            self.logs.clear();
            self.telemetry_logs.clear();
        }
    }
    
    impl Logger for TestLogger {
        fn log(&self, level: LogLevel, message: &str) {
            let mut self_mut = unsafe { &mut *(self as *const Self as *mut Self) };
            self_mut.logs.push((level, message.to_string()));
        }
        
        fn log_telemetry(&self, snapshot: TelemetrySnapshot, state: RuntimeState, temp: i16, timestamp: u64, inference_count: u32) {
            let mut self_mut = unsafe { &mut *(self as *const Self as *mut Self) };
            self_mut.telemetry_logs.push((snapshot, state, temp, timestamp, inference_count));
        }
    }
    
    pub fn test_safety_driven_fsm() -> Result<(), &'static str> {
        let mut clock = TestClock::new();
        let monotonic = MonotonicClock::new(clock.clone());
        let mut watchdog = TestWatchdog::new(monotonic, 1_000_000);
        let logger = TestLogger::new();
        
        let requirements = [
            safety::SafetyRequirement {
                id: 1,
                description: "Temperature limit",
                condition: safety::SafetyCondition::Temperature { max: MAX_TEMP_DECICELSIUS },
                max_faults: 1,
                fault_action: safety::FaultAction::EmergencyStop,
            },
            safety::SafetyRequirement {
                id: 2,
                description: "Miss rate limit",
                condition: safety::SafetyCondition::Rate { max_perthousand: 50 },
                max_faults: 5,
                fault_action: safety::FaultAction::ReduceLoad,
            },
            safety::SafetyRequirement {
                id: 3,
                description: "Checksum valid",
                condition: safety::SafetyCondition::Boolean,
                max_faults: 1,
                fault_action: safety::FaultAction::Shutdown,
            },
        ];
        
        let mut system = AiotSystem::new(requirements, clock.clone(), watchdog, logger);
        
        system.initialize().map_err(|_| "Init failed")?;
        
        system.process_temperature(900).map_err(|_| "Temp failed")?;
        assert!(matches!(system.state_machine.runtime_state(), RuntimeState::EmergencyStop));
        
        system.process_temperature(600).map_err(|_| "Temp failed")?;
        assert!(matches!(system.state_machine.runtime_state(), RuntimeState::CoolingDown));
        
        system.process_temperature(150).map_err(|_| "Temp failed")?;
        assert!(matches!(system.state_machine.runtime_state(), RuntimeState::Normal));
        
        let result = system.run_inference([0, 0, 0]);
        assert!(result.is_ok());
        
        let snapshot = system.telemetry_snapshot();
        assert!(snapshot.total_samples > 0);
        
        Ok(())
    }
    
    pub fn test_watchdog_expiration() -> Result<(), &'static str> {
        let mut clock = TestClock::new();
        let monotonic = MonotonicClock::new(clock.clone());
        let mut watchdog = TestWatchdog::new(monotonic, 1_000_000);
        let logger = TestLogger::new();
        
        let requirements = [
            safety::SafetyRequirement {
                id: 1,
                description: "Temp",
                condition: safety::SafetyCondition::Temperature { max: 850 },
                max_faults: 1,
                fault_action: safety::FaultAction::EmergencyStop,
            },
        ];
        
        let mut system = AiotSystem::new(requirements, clock.clone(), watchdog, logger);
        
        system.initialize().map_err(|_| "Init failed")?;
        
        clock.advance(2_000_000);
        
        let result = system.process_temperature(500);
        assert!(matches!(result, Err(SystemError::WatchdogTimeout)));
        
        Ok(())
    }
}

// ===================== MAIN =====================
#[cfg(feature = "embassy")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> Result<(), SystemError> {
    let clock = EmbassyClock;
    let monotonic = MonotonicClock::new(clock);
    let watchdog = HardwareWatchdog::new(monotonic, WATCHDOG_TIMEOUT_MICROS);
    let logger = EmbassyLogger;
    
    let requirements = [
        safety::SafetyRequirement {
            id: 1,
            description: "Temperature limit",
            condition: safety::SafetyCondition::Temperature { max: MAX_TEMP_DECICELSIUS },
            max_faults: 3,
            fault_action: safety::FaultAction::EmergencyStop,
        },
        safety::SafetyRequirement {
            id: 2,
            description: "Miss rate limit",
            condition: safety::SafetyCondition::Rate { max_perthousand: 50 },
            max_faults: 5,
            fault_action: safety::FaultAction::ReduceLoad,
        },
        safety::SafetyRequirement {
            id: 3,
            description: "Checksum valid",
            condition: safety::SafetyCondition::Boolean,
            max_faults: 1,
            fault_action: safety::FaultAction::Shutdown,
        },
    ];
    
    let mut system = AiotSystem::new(requirements, clock, watchdog, logger);
    
    system.initialize()?;
    
    for i in 1..=100 {
        let temp = 250 + (((i as f32) * 0.1).sin() * 100.0) as i16;
        system.process_temperature(temp)?;
        
        if system.state_machine.can_do_inference() {
            let _ = system.run_inference([100, 200, 300]);
        }
        
        embassy_time::Timer::after(Duration::from_millis(10)).await;
    }
    
    Ok(())
}

#[cfg(not(any(feature = "std", feature = "embassy")))]
fn main() {
    compile_error!("Enable 'embassy' feature for embedded or 'std' for testing");
}
