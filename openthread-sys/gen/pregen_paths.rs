use std::path::PathBuf;

use crate::builder::OpenThreadConfig;

pub struct PreGenerationPaths {
    pub bindings_rs_file: PathBuf,
    pub libs_dir: PathBuf,
    pub config_summary_file: PathBuf,
}

impl PreGenerationPaths {
    pub fn derive(
        crate_root_path: &PathBuf,
        target: &str,
        openthread_config: &OpenThreadConfig,
    ) -> PreGenerationPaths {
        let config_hash = openthread_config.config_hash();
        let config_based_path = crate_root_path
            .join("pre-generated")
            .join(format!("{config_hash}"));

        let config_summary_file = config_based_path.join("config.txt");

        let target_based_path = config_based_path.join(target);
        let bindings_rs_file = target_based_path.join("bindings.rs");
        let libs_dir = target_based_path.join("libs");

        PreGenerationPaths {
            bindings_rs_file,
            libs_dir,
            config_summary_file,
        }
    }
}
