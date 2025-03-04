use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::process::Command;

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run tests with different feature combinations
    TestFeatures,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::TestFeatures => test_features(),
    }
}

fn test_features() -> Result<()> {
    // Base features to always include
    let base_features = [];

    // Define sensor variants
    let sensor_variants = ["sen63c", "sen65", "sen66", "sen68"]; // "sen60" not supported yet

    // Define optional features
    let optional_features = ["async"];

    // Test each sensor variant with and without optional features
    for &sensor in &sensor_variants {
        // Test sensor variant alone
        println!("Testing with {}", sensor);
        run_test(
            &[sensor]
                .into_iter()
                .chain(base_features.iter().copied())
                .collect::<Vec<_>>(),
        )?;

        // Test sensor variant with each optional feature
        for &opt_feature in &optional_features {
            println!("Testing with {} and {}", sensor, opt_feature);
            run_test(
                &[sensor, opt_feature]
                    .into_iter()
                    .chain(base_features.iter().copied())
                    .collect::<Vec<_>>(),
            )?;
        }
    }

    println!("All tests passed!");
    Ok(())
}

fn run_test(features: &[&str]) -> Result<()> {
    let features_arg = features.join(",");
    println!("Running tests with features: {}", features_arg);

    let status = Command::new("cargo")
        .args([
            "test",
            "--no-default-features",
            "--lib",
            "--features",
            &features_arg,
        ])
        .status()
        .context("Failed to execute cargo test")?;

    if !status.success() {
        anyhow::bail!("Tests failed with features: {}", features_arg);
    }

    Ok(())
}
