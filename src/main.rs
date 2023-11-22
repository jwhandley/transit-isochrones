use chrono::NaiveTime;
use clap::Parser;
use kdtree::KdTree;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use transit_isochrones::{alpha_shape, build_graph_osm, dijkstra, geometry_to_geojson, Graph};

#[derive(Parser, Debug)]
struct Args {
    osm_path: String,
    gtfs_path: String,
    lat: f64,
    lon: f64,
    arrival_time: NaiveTime,
    duration: u32,
}

fn generate_isochrone(
    graph: &Graph,
    tree: &KdTree<f64, i64, [f64; 2]>,
    start_coords: &[f64; 2],
    arrival_time: NaiveTime,
    duration: u32,
) -> Result<String, anyhow::Error> {
    let accessible_points = dijkstra(graph, tree, start_coords, arrival_time, duration);
    let geom = alpha_shape(&accessible_points, 75.0)?;

    let geojson = geometry_to_geojson(geom)?;
    Ok(geojson)
}

fn calculate_hash<T: Hash>(t: &T) -> u64 {
    let mut s = DefaultHasher::new();
    t.hash(&mut s);
    s.finish()
}

fn main() -> Result<(), anyhow::Error> {
    let args = Args::parse();
    dbg!(&args);

    let str_to_hash = args.osm_path.clone()
        + &args.gtfs_path
        + &args.arrival_time.to_string()
        + &args.duration.to_string();
    let hash = calculate_hash(&str_to_hash);

    // Build the graph
    let (graph, tree) = build_graph_osm(&args.osm_path, &args.gtfs_path);

    // Run Dijkstra
    let start_time = std::time::Instant::now();
    let geojson = generate_isochrone(
        &graph,
        &tree,
        &[args.lon, args.lat],
        args.arrival_time,
        args.duration,
    )?;

    // Save the isochrone to a file
    std::fs::write(format!("data/isochrone_{}.geojson", hash), geojson)?;
    println!("Saved to data/isochrone_{}.geojson", hash);
    println!(
        "Took {}ms to generate isochrone",
        start_time.elapsed().as_millis()
    );

    Ok(())
}
