/// Global Project Constants
use crate::particle_filter::InitialDistributionType;
use lazy_static::lazy_static;
use nalgebra::Vector2;
use rand_distr::{Bernoulli, Normal, Poisson, Uniform};
use std::collections::HashMap;

// Simulation Parameters
#[allow(non_upper_case_globals)]
pub const x0: Vector2<f32> = Vector2::new(10.0, -10.0);
#[allow(non_upper_case_globals)]
pub const dt: f32 = 0.03333;
pub const END_TIME: f32 = 20.0;
pub const CLUTTER_AMOUNT: usize = 35;

// Random Number Generation Parameters and Distributions
pub const POISSON_MEAN: f32 = 2.0;
pub const GAUSSIAN_MEAN: f32 = 0.0;
pub const GAUSSIAN_STANDARD_DEVIATION: f32 = 1.5;

lazy_static! {
    // TODO: rename a bunch of these
pub static ref POISSON_DISTRIBUTION: Poisson<f32> = Poisson::new(POISSON_MEAN).unwrap();
pub static ref GAUSSIAN_DISTRIBUTION: Normal<f32> = Normal::new(GAUSSIAN_MEAN, GAUSSIAN_STANDARD_DEVIATION).unwrap();
pub static ref ARTIFICIAL_PROCESS_NOISE: Uniform<f32> = Uniform::new(-3.0, 3.0);
pub static ref MODEL_CHANGE: Bernoulli = Bernoulli::new(0.01).unwrap();
}

// Particle filter Parameters
pub const INITIAL_NUM_PARTICLES: usize = 500; 
pub const A_FEW_PARTICLES: usize = 5;
pub const INITIAL_ERROR_BOUND: f32 = 50.0;
pub const INITIAL_DISTRIBUTION_TYPE: InitialDistributionType = InitialDistributionType::UNIFORM;
pub const VALID_MODELS: [u8; 3] = [1, 2, 3];
pub const JUMPABLE_MODELS: [u8; 2] = [1, 2];

// Animation Parameters
pub const ANIMATION_FILENAME: &str = "animation.gif";
pub const BACKGROUND_COLOR: [u8; 3] = [0, 0, 0]; //[30, 17, 43];

lazy_static! {
    pub static ref COLORS: HashMap<&'static str, [u8; 3]> = [
        ("orange", [171, 111, 14]),
        ("blue", [50, 109, 168]),
        ("red", [168, 58, 50]),
        ("green", [50, 168, 109]),
        ("dark_matt_pink", [18, 3, 11]),
    ]
    .iter()
    .cloned()
    .collect();

    /// Map for deciding the radius of each plotted type of point
    pub static ref COLOR_RADIUS_MAP: HashMap<&'static str, usize> = [
        ("orange", 2),          // estimates
        ("blue", 1),            // particles
        ("red", 1),             // object measurments
        ("green", 1),           // true object state
        ("dark_matt_pink", 1),  // clutter or noise or false measurments
    ]
    .iter()
    .cloned()
    .collect();

    /// Map where you can add any color you want to disable the drawing of in the output gif.
    pub static ref DONT_DRAW: HashMap<&'static str, bool> = [
        //("blue", true),
        //("green", true),
        //("orange", true),
    ].iter().cloned().collect();
}

pub const GRID_SIZE: (i32, i32) = (100, 100); // (row, col)
pub const ORIGIN: (i32, i32) = (50, 50); // (row, col)
pub const NUM_CHANNELS: i32 = 3;
pub const FLAT_ARRAY_SIZE: usize = (GRID_SIZE.0 * GRID_SIZE.1 * NUM_CHANNELS) as usize;
