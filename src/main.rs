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
    print_start_message();
    print!("\n\nSimulating...");
    io::stdout().flush().unwrap();

    let mut data_giffer = anim::DataGiffer::new();
    let rng = &mut thread_rng();

    let mut system = sys::Sys {x: consts::x0, f: sys::example_hybrid_system, h: sys::measurement_function};
    let mut system2 = sys::Sys {x: consts::x0 + Vector2::new(10.0, -7.0), f: sys::example_hybrid_system, h: sys::measurement_function};

    let mut m: u8 = 1; // TODO make vector of modes instead
    let mut m2: u8 = 2;

    let mut t: f32 = 0.0;

    let mut y       = vec![(system.h)(t, system.x, m, rng), (system2.h)(t, system2.x, m2, rng)];
    let mut clutter = sys::clutter(consts::CLUTTER_AMOUNT, rng);
    y.extend_from_slice(&clutter);

    let mut filter = particle_filter::ParticleFilter::initialize(&y, rng);
    
    for i in 0..((consts::END_TIME/consts::dt) as usize) {
        
        t += consts::dt;

        let (points_to_draw, ordered_colors) = color_helper(&vec![&clutter, &samples_to_points_helper(filter.samples), &y, &filter.estimates, &vec![system.x, system2.x]]);
        data_giffer.draw_points(&points_to_draw, &ordered_colors);

        filter.predict(t, consts::dt, rng);

        sim::step(t, &mut system, m, consts::dt);
        sim::step(t, &mut system2, m2, consts::dt);

        y = vec![(system.h)(t, system.x, m, rng), (system2.h)(t, system2.x, m2, rng)];
        clutter = sys::clutter(consts::CLUTTER_AMOUNT, rng);
        y.extend_from_slice(&clutter);

        filter.update(&y, rng);
        filter.resample(&y, rng);

        m = sys::true_model_change_posterior(m, rng); 
        m2 = sys::true_model_change_posterior(m2, rng);
        
    }
    
    print!("\n\nDone simulating.\n\nExporting animation...");
    io::stdout().flush().unwrap();
    
    data_giffer.export_gif();
    print!("\n\nDone exporting. File: "); print!("{}\n", consts::ANIMATION_FILENAME);

} // END main()



/// Pass a Vec of Vec of Vector2 and receive a concatenated Vec of Vector2 and a Vec of &str that
/// can be used to get RGB values for colors. Each element of the concatenated Vec then has the same
/// color as the other elements in the original (inner) Vec it came from.
fn color_helper<'a>(points_to_draw: &Vec<&Vec<Vector2<f32>>>) -> (Vec<Vector2<f32>>, Vec<&'a str>) {
    let availale_colors: Vec<&str> = vec!["matt_pink", "blue", "red", "orange", "green"];
    let mut ordered_colors: Vec<&str> = Vec::new();
    let mut concatenated_vector: Vec<Vector2<f32>> = Vec::new();
    
    let mut i = 0;
    for &point_set in points_to_draw {
        for point in point_set {
            ordered_colors.push(&availale_colors[i]); 
        }
        concatenated_vector.extend_from_slice(&point_set);
        i = if i < availale_colors.len() { i + 1 } else { i };
    }
    return (concatenated_vector, ordered_colors);
}

/// Just extracts the vectors/points we want to draw
fn samples_to_points_helper(samples: [(Vector2<f32>, u8); consts::INITIAL_NUM_PARTICLES]) -> Vec<Vector2<f32>> {
    let mut point_vector: Vec<Vector2<f32>> = Vec::with_capacity(consts::INITIAL_NUM_PARTICLES);
    for (point, _) in samples {
            point_vector.push(point);
    }
    return point_vector;
}

/// A nice little info message
fn print_start_message() {
    print!("\n\nFilter parameters:\n\nNumber of particles: {}", consts::INITIAL_NUM_PARTICLES);
    print!("\nAmount of clutter at all times: {}", consts::CLUTTER_AMOUNT);
    print!("\nInitial distribution type: {}", consts::INITIAL_DISTRIBUTION_TYPE.to_string());
    print!("\nSimulation endtime: {}", consts::END_TIME);
    print!("\nSimulation timestep (dt): {}", consts::dt);
    print!("\nSimulation method: {}", "ERK4");
    print!("\n\nAnimation legend:\n\nTrue state: Green\nEstimated state: Orange\nParticles: Blue\nObject oriented measurements: Red\nClutter: Matt Pink");
    print!("\n\n");
    io::stdout().flush().unwrap();
}
