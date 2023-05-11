/// Defines a dynamic system

use nalgebra::Vector2;
use rand::rngs::ThreadRng;
use rand_distr::Distribution;

pub struct Sys {
    pub x: Vector2<f32>,                                // state
    pub f: fn(f32, Vector2<f32>, u8) -> Vector2<f32>,   // x_dot = f(t, x, m) continuous time dynamic system function
    pub h: fn(f32, Vector2<f32>, u8, &mut ThreadRng) -> Vector2<f32>,
}

pub fn measurement_function(t: f32, x: Vector2<f32>, m: u8, rng: &mut ThreadRng) -> Vector2<f32> {
    let mut noise: Vector2<f32> = Vector2::new(0.0, 0.0);
    match m {
        1 => { 
            noise[0] = crate::consts::POISSON_DISTRIBUTION.sample(rng);
            noise[1] = crate::consts::POISSON_DISTRIBUTION.sample(rng);
        }
        2 => { 
            noise[0] = crate::consts::GAUSSIAN_DISTRIBUTION.sample(rng); 
            noise[1] = crate::consts::GAUSSIAN_DISTRIBUTION.sample(rng); 
        }
        _ => { print!("Undefined mode.\n"); }
    }
    return x + noise; // TODO: give back multiple measurements and add background noise
}


pub fn circle_path(t: f32) -> Vector2<f32> {
    const A: f32 = 20.0;
    const W: f32 = 8.5;
    return Vector2::new(A*f32::cos(W*t), A*f32::sin(W*t));
}

pub fn figure_eight_path(t: f32) -> Vector2<f32> {
    const A: f32 = 15.0;
    const W: f32 = 11.3;
    return Vector2::new(A*f32::cos(W*t), A*f32::sin(W*t)*f32::cos(W*t));
}


#[allow(non_upper_case_globals)]
pub fn example_hybrid_system(t: f32, x: Vector2<f32>, m: u8) -> Vector2<f32> {
    const u1_max: f32 = 5.00;
    const u2_max: f32 = 8.50; 
    const a1: f32 = 0.03;
    const a2: f32 = 0.08;

    // A basic proportional controller trying to follow some reference paths
    const zero_vector: Vector2<f32> = Vector2::new(0.0, 0.0);
    let e: Vector2<f32> = if m == 1 {x - circle_path(t) } else if m == 2 {x - figure_eight_path(t) } else { zero_vector };
    let kp_1: f32 = 1.0;
    let kp_2: f32 = 2.0;

    let mut x_dot: Vector2<f32> = zero_vector;
    match m {
        1 => { 
            x_dot[0] = a1*x[0] + f32::sin(0.5*x[1]) - f32::max(-u1_max, f32::min(u1_max, kp_1*e[0]));
            x_dot[1] = a1*x[1] + f32::cos(0.4*x[1]) - f32::max(-u1_max, f32::min(u1_max, kp_1*e[1]));
        }
        2 => {
            x_dot[0] = a2*x[0] + f32::sin(0.1*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[0]));
            x_dot[1] = a2*x[1] + f32::cos(0.2*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[1]));
        }
        _ => { print!("Undefined mode.\n"); }
    }
    return x_dot;
}

//pub fn motion_model_1(t: f32, x: Vector2<f32>, m: u8) -> Vector2<f32> {
//    const a: f32 = 0.5;
//    let e: f32 = if m == 1 { x - circle_path(t); }
//   return a*(-x); // x_dot = a
//}
