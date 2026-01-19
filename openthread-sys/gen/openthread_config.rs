use std::{
    collections::{btree_map::Iter, BTreeMap},
    fmt::Display,
    hash::{DefaultHasher, Hash, Hasher},
};

#[derive(Clone)]
pub struct OpenThreadConfig {
    config_values: BTreeMap<&'static str, &'static str>,
}

impl OpenThreadConfig {
    pub fn app_cli(&mut self, enable: bool) {
        self.set_boolean("OT_APP_CLI", enable);
    }

    pub fn app_ncp(&mut self, enable: bool) {
        self.set_boolean("OT_APP_NCP", enable);
    }

    pub fn app_rcp(&mut self, enable: bool) {
        self.set_boolean("OT_APP_RCP", enable);
    }

    pub fn border_router(&mut self, enable: bool) {
        self.set_boolean("OT_BORDER_ROUTER", enable);
    }

    pub fn border_routing(&mut self, enable: bool) {
        self.set_boolean("OT_BORDER_ROUTING", enable);
    }

    pub fn ecdsa(&mut self, enable: bool) {
        self.set_boolean("OT_ECDSA", enable);
    }

    pub fn ftd(&mut self, enable_ftd: bool) {
        self.set_boolean("OT_MTD", !enable_ftd);
        self.set_boolean("OT_FTD", enable_ftd);
    }

    pub fn log_level(&mut self, log_level: OpenThreadLogLevel) {
        use OpenThreadLogLevel::*;

        self.set_string(
            "OT_LOG_LEVEL",
            match log_level {
                None => "NONE",
                Crit => "CRIT",
                Warn => "WARN",
                Note => "NOTE",
                Info => "INFO",
                Debg => "DEBG",
            },
        );
    }

    fn log_output(&mut self, log_output: OpenThreadLogOutput) {
        use OpenThreadLogOutput::*;

        self.set_string(
            "OT_LOG_OUTPUT",
            match log_output {
                App => "APP",
                DebugUart => "DEBUG_UART",
                None => "NONE",
                PlatformDefined => "PLATFORM_DEFINED",
            },
        );
    }

    pub fn ping_sender(&mut self, enable: bool) {
        self.set_boolean("OT_PING_SENDER", enable);
    }

    fn platform(&mut self, platform: OpenThreadPlatform) {
        use OpenThreadPlatform::*;

        self.set_string(
            "OT_PLATFORM",
            match platform {
                External => "external",
            },
        );
    }

    pub fn rcp(&mut self, enable: bool) {
        self.set_boolean("OT_RCP", enable);
    }

    fn settings_ram(&mut self, enable: bool) {
        self.set_boolean("OT_SETTINGS_RAM", enable);
    }

    pub fn slaac(&mut self, enable: bool) {
        self.set_boolean("OT_SLAAC", enable);
    }

    pub fn srp_client(&mut self, enable: bool) {
        self.set_boolean("OT_SRP_CLIENT", enable);
    }

    pub fn tcp(&mut self, enable: bool) {
        self.set_boolean("OT_TCP", enable);
    }

    fn set_boolean(&mut self, key: &'static str, value: bool) {
        self.set_string(key, if value { "ON" } else { "OFF" });
    }

    fn set_string(&mut self, key: &'static str, value: &'static str) {
        self.config_values.entry(key).insert_entry(value);
    }

    pub fn iter(&self) -> Iter<'_, &str, &str> {
        self.config_values.iter()
    }

    pub fn config_hash(&self) -> u64 {
        let mut hasher = DefaultHasher::new();
        self.hash(&mut hasher);
        hasher.finish()
    }
}

impl Default for OpenThreadConfig {
    fn default() -> Self {
        let mut config = Self {
            config_values: BTreeMap::new(),
        };

        config.log_level(OpenThreadLogLevel::Note);
        config.ftd(false);
        config.rcp(false);
        config.tcp(false);
        config.app_cli(false);
        config.app_ncp(false);
        config.app_rcp(false);
        config.border_router(false);
        config.border_routing(false);
        config.srp_client(true);
        config.slaac(true);
        config.ecdsa(true);
        config.ping_sender(true);
        config.log_output(OpenThreadLogOutput::PlatformDefined);
        config.platform(OpenThreadPlatform::External);
        config.settings_ram(false);

        config
    }
}

impl Hash for OpenThreadConfig {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        for (key, value) in self.iter() {
            format!("{key}={value}").hash(state);
        }
    }
}

impl Display for OpenThreadConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for (key, value) in self.iter() {
            f.write_str(key)?;
            f.write_str(" = ")?;
            f.write_str(value)?;
            f.write_str("\n")?;
        }

        Ok(())
    }
}

#[derive(Hash)]
#[allow(dead_code)]
pub enum OpenThreadLogLevel {
    None,
    Crit,
    Warn,
    Note,
    Info,
    Debg,
}

#[derive(Hash)]
#[allow(dead_code)]
pub enum OpenThreadLogOutput {
    App,
    DebugUart,
    None,
    PlatformDefined,
}

#[derive(Hash)]
pub enum OpenThreadPlatform {
    External,
}
