/// Defines a dynamic system
pub mod sys {

use nalgebra::Vector2;

pub struct Sys {
    x: Vector2<f32>, // state
    f: fn(Vector2<f32>) -> Vector2<f32>,      // x_dot = f(x) continuous time dynamic system function
}

#[allow(non_upper_case_globals)]
pub fn example_hybrid_system(x: Vector2<f32>, m: u8) -> Vector2<f32> {
    const u1_max: f32 = 0.10;
    const u2_max: f32 = 1.00; 
    const a1: f32 = 1.3;
    const a2: f32 = 0.8;

    // Some random proportional controller
    const x_ref_1: Vector2<f32> = Vector2::new(10.0, 20.0);
    const x_ref_2: Vector2<f32> = Vector2::new(-10.0, 3.0);
    const zero_vector: Vector2<f32> = Vector2::new(0.0, 0.0);
    let e: Vector2<f32> = if m == 1 {x - x_ref_1} else if m == 2 {x - x_ref_2} else { zero_vector };
    let kp_1: f32 = 1.0;
    let kp_2: f32 = 2.0;

    let mut x_dot: Vector2<f32> = zero_vector;
    match m {
        0 => { 
            x_dot[0] = a1*x[0] + f32::sin(x[1]) - f32::min(u1_max, kp_1*e[0]);
            x_dot[1] = a1*x[1] + f32::cos(x[1]) - f32::min(u1_max, kp_1*e[1]);
        }
        1 => {
            x_dot[0] = a2*x[0] + f32::sin(x[1]) - f32::min(u2_max, kp_2*e[0]);
            x_dot[1] = a2*x[1] + f32::cos(x[1]) - f32::min(u2_max, kp_2*e[1]);
        }
        _ => {
            print!("Undefined mode.\n");
        }
    }
    return x_dot;
}

} // end mod
