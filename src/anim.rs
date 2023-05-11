/// TODO
use nalgebra::Vector2;
use gif::{Frame, Encoder, Repeat};
use std::fs::File;

pub struct DataGiffer {
    default_frame: [u8; crate::consts::FLAT_ARRAY_SIZE],
    frames: Vec<[u8; crate::consts::FLAT_ARRAY_SIZE]>,
    size: (i32, i32),
    origin: (i32, i32),
}

fn within_frame(point: (i32, i32), size: (i32, i32)) -> bool {
    return 0 <= point.0 && point.0 < size.0 && 0 <= point.1 && point.1 < size.1;
}

impl DataGiffer {

    pub fn new() -> Self {
        let new_data_giffer = DataGiffer{
            default_frame : Self::create_default_frame(),
            frames: Vec::default(),
            size : crate::consts::GRID_SIZE,
            origin : crate::consts::ORIGIN,
        };
        assert!(new_data_giffer.size.0 >= 0 && new_data_giffer.size.1 >= 0);
        assert!(within_frame(new_data_giffer.origin, new_data_giffer.size));
        return new_data_giffer;
    }

    fn create_default_frame() -> [u8; crate::consts::FLAT_ARRAY_SIZE] {
        let mut default_frame:  [u8; crate::consts::FLAT_ARRAY_SIZE] = [0; crate::consts::FLAT_ARRAY_SIZE];
        assert!(crate::consts::NUM_CHANNELS as usize == crate::consts::BACKGROUND_COLOR.len());

        for i in 0..crate::consts::FLAT_ARRAY_SIZE {
            default_frame[i] = crate::consts::BACKGROUND_COLOR[i % crate::consts::NUM_CHANNELS as usize];
        }
        return default_frame;
    }
 
    pub fn draw_points(&mut self, state_vecs: &Vec<Vector2<f32>>, color_indices: &Vec<&str>) -> bool {
        let mut all_inside: bool = true;
        let frame = &mut self.default_frame.clone();

        for (vector, color) in state_vecs.iter().zip(color_indices.iter()) {

            // FIXME: would probably be better to do positivity check and type casting here and keep size, origin and NUM_CHANNELS as usize
            let pixel_point: (i32, i32) = (f32::round(vector[0]) as i32 + self.origin.0, f32::round(vector[1]) as i32 + self.origin.1); 
            
            if !within_frame(pixel_point, self.size) {
                all_inside = false;
            } else {
                self.draw_point(pixel_point, frame, color);
            }
        }
        self.frames.push(*frame);
        return all_inside;
    }

    fn draw_point(&self, pixel_point: (i32, i32), frame: &mut [u8], color: &str) {
        assert!(pixel_point.0 >= 0 && pixel_point.1 >= 0);
        let y = pixel_point.0;
        let x = pixel_point.1;
        let m = self.size.1;
        let c = crate::consts::NUM_CHANNELS;
        let flat_rgb_pixel_index_range = (y*m*c + x*c) as usize .. (y*m*c + x*c + c) as usize;
        frame[ flat_rgb_pixel_index_range ].copy_from_slice(crate::consts::COLORS.get(color).unwrap()); 
    }

    pub fn export_gif(&self) {
        let mut file = File::create(crate::consts::ANIMATION_FILENAME).unwrap();
        let mut encoder = Encoder::new(&mut file, self.size.0 as u16, self.size.1 as u16, &[]).unwrap();
        encoder.set_repeat(Repeat::Infinite).unwrap( );
        
        for frame in &self.frames {
            let gif_frame = &mut Frame::from_rgb(self.size.0 as u16, self.size.1 as u16, frame);
            gif_frame.delay = u16::max((crate::consts::dt as u16) * 100, 1);
            encoder.write_frame(&Frame::from_rgb(self.size.0 as u16, self.size.1 as u16, frame)).unwrap();
        }
    }
}
