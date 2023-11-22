use chrono::NaiveTime;
use clap::Parser;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
mod dijkstra;
mod graph;
mod isochrone;
use crate::dijkstra::dijkstra;
use crate::graph::{build_graph_osm, Graph};
use crate::isochrone::{alpha_shape, geometry_to_geojson};

#[derive(Parser, Debug)]
struct Args {
    osm_path: String,
    gtfs_path: String,
    arrival_time: NaiveTime,
    duration: u32,
}

fn main() -> Result<(), anyhow::Error> {
    let args = Args::parse();

    let str_to_hash = args.osm_path.clone()
        + &args.gtfs_path
        + &args.arrival_time.to_string()
        + &args.duration.to_string();
    let hash = calculate_hash(&str_to_hash);

    // Build the graph
    let graph = build_graph_osm(&args.osm_path, &args.gtfs_path);

    // Run Dijkstra
    let start_time = std::time::Instant::now();
    let points = dijkstra(
        &graph,
        &[2.356693437528564, 48.88230685842714],
        args.arrival_time,
        args.duration,
    );
    dbg!(points.len());
    println!(
        "Took {}ms to run Dijkstra",
        start_time.elapsed().as_millis()
    );

    // Create isochrone
    let start_time = std::time::Instant::now();
    let geom = alpha_shape(&points, 75.0)?;
    let geojson = geometry_to_geojson(geom)?;
    println!(
        "Took {}ms to create the isochrone",
        start_time.elapsed().as_millis()
    );

    // Save the isochrone to a file
    std::fs::write(format!("data/isochrone_{}.geojson", hash), geojson)?;
    println!("Saved to data/isochrone_{}.geojson", hash);

    Ok(())
}

fn calculate_hash<T: Hash>(t: &T) -> u64 {
    let mut s = DefaultHasher::new();
    t.hash(&mut s);
    s.finish()
}
