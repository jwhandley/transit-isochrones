use crate::dijkstra::haversine_distance;
use delaunator::{triangulate, Point};
use geo::Coord;
use geojson::Feature;
use geojson::GeoJson;
use geojson::Geometry as JsonGeometry;
use geos::Geom;
use geos::Geometry;
use kdtree::KdTree;
use std::cmp::max;
use std::cmp::min;
use std::collections::HashMap;

const OFF_ROAD_WALKING_SPEED: f64 = 1.0;

pub fn create_grid(
    midpoint: &[f64; 2],
    size: f64,
    resolution: usize,
    tree: &KdTree<f64, u32, [f64; 2]>,
    duration: u32,
) -> Vec<[f64; 2]> {
    let mut grid = Vec::with_capacity(resolution * resolution);

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

            if cost < duration as f64 {
                grid.push([x, y]);
            }
        }
    }
    grid
}

/// Converts coordinates to delaunator points.
fn convert_points(points: &[[f64; 2]]) -> Vec<Point> {
    points.iter().map(|p| Point { x: p[0], y: p[1] }).collect()
}

/// Returns the alpha shape of a set of points.
pub fn alpha_shape(points: &[[f64; 2]], alpha: f64) -> Result<Geometry, anyhow::Error> {
    let points = &convert_points(points);
    let start_time = std::time::Instant::now();
    let triangulation = triangulate(points);
    println!("Took {}ms to triangulate", start_time.elapsed().as_millis());

    // Trace out edges of the triangulation
    let start_time = std::time::Instant::now();
    let mut edges_count = HashMap::new();
    for i in 0..triangulation.triangles.len() / 3 {
        let i0 = triangulation.triangles[3 * i];
        let i1 = triangulation.triangles[3 * i + 1];
        let i2 = triangulation.triangles[3 * i + 2];

        let p0 = &points[i0];
        let p1 = &points[i1];
        let p2 = &points[i2];

        let circumradius = calculate_circumradius(p0, p1, p2);

        if circumradius < alpha {
            *edges_count.entry((min(i0, i1), max(i0, i1))).or_insert(0) += 1;
            *edges_count.entry((min(i1, i2), max(i1, i2))).or_insert(0) += 1;
            *edges_count.entry((min(i2, i0), max(i2, i0))).or_insert(0) += 1;
        }
    }
    println!("Took {}ms to trace edges", start_time.elapsed().as_millis());

    // Convert edges to a linestring
    let start_time = std::time::Instant::now();
    let mut linestrings = vec![];
    for (&(i0, i1), _) in edges_count.iter().filter(|&(_, v)| *v == 1) {
        let p0 = &points[i0];
        let p1 = &points[i1];
        let line: geos::Geometry =
            geo::LineString::new(vec![Coord::from([p0.x, p0.y]), Coord::from([p1.x, p1.y])])
                .try_into()?;
        linestrings.push(line);
    }

    println!(
        "Took {}ms to convert edges to linestrings",
        start_time.elapsed().as_millis()
    );

    // Convert linestring to polygon
    let start_time = std::time::Instant::now();
    let polygon = Geometry::polygonize(&linestrings)?.unary_union()?;
    println!(
        "Took {}ms to convert linestrings to polygon",
        start_time.elapsed().as_millis()
    );

    Ok(polygon)
}

fn calculate_circumradius(p1: &Point, p2: &Point, p3: &Point) -> f64 {
    let d1 = haversine_distance(&[p1.x, p1.y], &[p2.x, p2.y]);
    let d2 = haversine_distance(&[p2.x, p2.y], &[p3.x, p3.y]);
    let d3 = haversine_distance(&[p3.x, p3.y], &[p1.x, p1.y]);

    let s = (d1 + d2 + d3) / 2.0;
    let area = (s * (s - d1) * (s - d2) * (s - d3)).sqrt();

    d1 * d2 * d3 / (4.0 * area)
}

pub fn geometry_to_geojson(geometry: Geometry) -> Result<String, anyhow::Error> {
    let geometry: JsonGeometry = geometry.try_into()?;

    let geojson = GeoJson::Feature(Feature {
        bbox: None,
        geometry: Some(geometry),
        id: None,
        properties: None,
        foreign_members: None,
    });

    Ok(geojson.to_string())
}
