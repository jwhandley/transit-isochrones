use contour::ContourBuilder;
use geojson::FeatureCollection;
use geojson::GeoJson;
use kdtree::KdTree;
use rayon::prelude::*;

use crate::graph::nearest_point;
const OFF_ROAD_WALKING_SPEED: f64 = 1.0;
const DEGREES_TO_METERS: f64 = 111_111.0;

pub fn create_contour(
    midpoint: &[f64; 2],
    size: f64,
    resolution: usize,
    tree: &KdTree<f64, u32, [f64; 2]>,
    duration: u32,
) -> Result<String, anyhow::Error> {
    let dlat = size / DEGREES_TO_METERS / resolution as f64; // 1 degree latitude is 111111 meters
    let dlon = size / (DEGREES_TO_METERS * midpoint[1].to_radians().cos()) / resolution as f64; // Adjusting for longitude, considering latitude

    let half_grid_size = size / 2.0;
    let min_lat = midpoint[1] - half_grid_size / DEGREES_TO_METERS;
    let min_lon =
        midpoint[0] - half_grid_size / (DEGREES_TO_METERS * midpoint[1].to_radians().cos());

    let values = (0..resolution)
        .into_par_iter()
        .flat_map(|i| {
            (0..resolution).into_par_iter().map(move |j| {
                let x = min_lon + dlon * i as f64;
                let y = min_lat + dlat * j as f64;

                let (distance, time) = nearest_point(&tree, &[x, y]).unwrap();

                let cost = time as f64 + distance / OFF_ROAD_WALKING_SPEED;

                -cost
            })
        })
        .collect::<Vec<f64>>();

    let features = ContourBuilder::new(resolution, resolution, true)
        .x_origin(min_lon)
        .y_origin(min_lat)
        .x_step(dlon)
        .y_step(dlat)
        .contours(&values, &[-(duration as f64)])?
        .iter()
        .map(|contour| contour.to_geojson())
        .collect::<Vec<geojson::Feature>>();

    let geojson_string = GeoJson::from(FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    })
    .to_string();

    Ok(geojson_string)
}
