use geojson::FeatureCollection;
use geojson::GeoJson;
use kdtree::KdTree;
use contour::ContourBuilder;
const OFF_ROAD_WALKING_SPEED: f64 = 1.0;

pub fn create_contour(
    midpoint: &[f64; 2],
    size: f64,
    resolution: usize,
    tree: &KdTree<f64, u32, [f64; 2]>,
    duration: u32,
) -> Result<String, anyhow::Error> {
    let mut values = Vec::with_capacity(resolution * resolution);

    let dlat = size / 111_111.0 / resolution as f64; // 1 degree latitude is 111111 meters
    let dlon = size / (111_111.0 * midpoint[1].to_radians().cos()) / resolution as f64; // Adjusting for longitude, considering latitude

    let half_grid_size = size / 2.0;
    let min_lat = midpoint[1] - half_grid_size / 111_111.0;
    let min_lon = midpoint[0] - half_grid_size / (111_111.0 * midpoint[1].to_radians().cos());

    for i in 0..resolution {
        for j in 0..resolution {
            let x = min_lon + dlon * i as f64;
            let y = min_lat + dlat * j as f64;

            let nearest = tree
                .nearest(&[x, y], 1, &crate::dijkstra::haversine_distance)
                .unwrap();

            let nearest_node = nearest[0];
            let time = *nearest_node.1 as f64;
            let distance = nearest_node.0;

            let cost = time + distance / OFF_ROAD_WALKING_SPEED;

            values.push(-cost);
        }
    }
    
    let features = ContourBuilder::new(resolution as u32, resolution as u32, true)
        .x_origin(min_lon)
        .y_origin(min_lat)
        .x_step(dlon)
        .y_step(dlat)
        .contours(&values, &[-(duration as f64)])?
        .iter()
        .map(|contour| contour.to_geojson())
        .collect::<Vec<geojson::Feature>>();
    
    let geojson_string = GeoJson::from(
        FeatureCollection {
            bbox: None,
            features,
            foreign_members: None,
        }).to_string();

    Ok(geojson_string)
}