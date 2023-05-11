mod sim; 
mod sys;
mod consts;
mod anim;
mod particle_filter;

use std::io;
use std::io::Write;

use nalgebra::Vector2;
use rand::thread_rng;

fn main() {
    print!("\n\nSimulating...");
    io::stdout().flush().unwrap();

    let mut data_giffer = anim::DataGiffer::new();
    let mut system = sys::Sys {x: consts::x0, f: sys::example_hybrid_system, h: sys::measurement_function};
    let rng = &mut thread_rng();

    let mut m: u8 = 1;
    let mut t: f32 = 0.0;

    let mut filter = particle_filter::ParticleFilter::initialize(&vec![(system.h)(t, system.x, m, rng)], rng);
    
    for i in 0..((consts::END_TIME/consts::dt) as usize) {
        
        t += consts::dt;
        if t >= (1.0/2.0)*consts::END_TIME { m = 2; }

        let y = (system.h)(t, system.x, m, rng);

        filter.predict(t, m, consts::dt, rng);
        filter.update(&vec![y], m, rng);
        filter.resample(rng);

        let (points_to_draw, ordered_colors) = color_helper(&vec![vec![system.x], vec![y], filter.samples.to_vec(), vec![filter.estimate]]);
        
        data_giffer.draw_points(&points_to_draw, &ordered_colors);
        sim::step(t, &mut system, m, consts::dt);
    }
    
    print!("\n\nDone simulating.\n\nExporting animation...");
    io::stdout().flush().unwrap();
    
    data_giffer.export_gif();
    print!("\n\nDone exporting.\n"); 
}

// Pass a Vec of Vec of Vector2 and receive a concatenated Vec of Vector2 and a Vec of &str that
// can be used to get RGB values for colors. Each element of the concatenated Vec then has the same
// color as the other elements in the original (inner) Vec it came from.
fn color_helper<'a>(points_to_draw: &Vec<Vec<Vector2<f32>>>) -> (Vec<Vector2<f32>>, Vec<&'a str>) {
    let availale_colors: Vec<&str> = vec!["green", "red", "blue", "orange"];
    let mut ordered_colors: Vec<&str> = Vec::new();
    let mut concatenated_vector: Vec<Vector2<f32>> = Vec::new();
    
    let mut i = 0;
    for point_set in points_to_draw {
        for point in point_set {
            ordered_colors.push(&availale_colors[i]); 
        }
        concatenated_vector.extend_from_slice(&point_set);
        i = if i < availale_colors.len() { i + 1 } else { i };
    }
    return (concatenated_vector, ordered_colors);
}
