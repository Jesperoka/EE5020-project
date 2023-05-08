/// Global Project Constants
use nalgebra::Vector2;

#[allow(non_upper_case_globals)]
pub const x0: Vector2<f32> = Vector2::new(0.0, 0.0);
#[allow(non_upper_case_globals)]
pub const dt: f32 = 0.03333; 
//pub const num_particles: u8 = 50;

// Animation Parameters
pub const ANIMATION_FILENAME: &str = "animation.gif";

pub const BACKGROUND_COLOR: [u8; 3] = [30, 17, 43];
pub const FOREGROUND_COLORS: [[u8; 3]; 1] = [[171, 111, 14]];

pub const GRID_SIZE: (i32, i32) = (100, 100);
pub const ORIGIN: (i32, i32) = (50, 50);
pub const NUM_CHANNELS: i32 = 3;
pub const FLAT_ARRAY_SIZE: usize = (GRID_SIZE.0 * GRID_SIZE.1 * NUM_CHANNELS) as usize;

