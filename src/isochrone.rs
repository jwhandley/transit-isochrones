use delaunator::{triangulate, Point};
use geojson::Feature;
use geojson::GeoJson;
use geojson::Geometry as JsonGeometry;
use geos::Geom;
use geos::Geometry;
use std::cmp::max;
use std::cmp::min;
use std::collections::HashSet;
use crate::dijkstra::haversine_distance;

pub fn create_grid(
    midpoint: &[f64; 2],
    step: f64,
    size: usize,
    points: &[[f64; 2]],
    threshold: f64,
) -> Vec<Point> {
    let mut grid = Vec::new();
    let x_min = midpoint[0] - step * (size / 2) as f64;
    let y_min = midpoint[1] - step * (size / 2) as f64;

    for i in 0..size {
        for j in 0..size {
            let x = x_min + step * i as f64;
            let y = y_min + step * j as f64;
            let point = Point { x, y };
            if idw(&point, points) > threshold {
                grid.push(point);
            }
        }
    }

    grid
}

fn idw(point: &Point, points: &[[f64; 2]]) -> f64 {
    let mut sum = 0.0;

    for p in points {
        let distance = haversine_distance(&[point.x, point.y], p);
        if distance == 0.0 {
            sum += 1.0;
        }
        sum += 1.0 / distance;
    }

    sum / points.len() as f64
}

pub fn convert_points(points: &[[f64;2]]) -> Vec<Point> {
    points.iter().map(|p| Point { x: p[0], y: p[1] }).collect()
}

/// Returns the alpha shape of a set of points.
pub fn alpha_shape(points: &[Point], alpha: f64) -> Result<Geometry, anyhow::Error> {
    let triangulation = triangulate(points);

    // Trace out edges of the triangulation
    let mut edges = HashSet::new();
    for i in 0..triangulation.triangles.len() / 3 {
        let i0 = triangulation.triangles[3 * i];
        let i1 = triangulation.triangles[3 * i + 1];
        let i2 = triangulation.triangles[3 * i + 2];

        let p0 = &points[i0];
        let p1 = &points[i1];
        let p2 = &points[i2];

        let circumradius = calculate_circumradius(p0, p1, p2);
        // dbg!(circumradius);

        if circumradius < alpha {
            edges.insert((min(i0, i1), max(i0, i1)));
            edges.insert((min(i1, i2), max(i1, i2)));
            edges.insert((min(i2, i0), max(i2, i0)));
        }
    }

    // Convert edges to a linestring
    let mut linestrings = vec![];
    for (i0, i1) in edges {
        let p0 = &points[i0];
        let p1 = &points[i1];
        let line = geos::Geometry::new_from_wkt(&format!(
            "LINESTRING({} {}, {} {})",
            p0.x, p0.y, p1.x, p1.y
        ))?;
        linestrings.push(line);
    }

    // Convert linestring to polygon
    let polygon = Geometry::polygonize(&linestrings)?.unary_union()?;

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
