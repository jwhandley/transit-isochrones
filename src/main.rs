#[macro_use]
extern crate rocket;
use std::path::PathBuf;

use chrono::NaiveTime;
use clap::Parser;
use kdtree::KdTree;
use rocket::{http::Status, State};
use transit_isochrones::{build_graph_osm, create_contour, dijkstra, Graph};

const GRID_SIZE: usize = 250; // Number of points per side of the grid
const ONE_HOUR: f64 = 3600.0; // Seconds per hour
const MAX_SPEED: f64 = 75_000.0; // Meters per hour

#[derive(Parser, Debug)]
struct Args {
    osm_path: PathBuf,
    gtfs_path: PathBuf,
}

#[get("/?<lat>&<lon>&<arrival_time>&<duration>")]
fn isochrone(
    graph: &State<Graph>,
    lat: f64,
    lon: f64,
    arrival_time: &str,
    duration: u32,
) -> (Status, String) {
    let start_coords = &[lon, lat];

    let arrival_time = match NaiveTime::parse_from_str(arrival_time, "%H:%M:%S") {
        Ok(time) => time,
        Err(_) => return (Status::BadRequest, "Invalid arrival time".to_string()),
    };

    let start_time = std::time::Instant::now();
    let reachable_nodes = match dijkstra(&graph, start_coords, arrival_time, duration) {
        Ok(tree) => tree,
        Err(_) => {
            return (
                Status::InternalServerError,
                "Error finding reachable nodes".to_string(),
            )
        }
    };

    println!(
        "Took {}ms to find {} reachable nodes",
        start_time.elapsed().as_millis(),
        reachable_nodes.len()
    );

    let start_time = std::time::Instant::now();
    let mut sptree = KdTree::new(2);
    for (node_id, dist) in reachable_nodes {
        let node = graph.get_node(&node_id).unwrap();
        sptree.add([node.lon, node.lat], dist).unwrap();
    }

    let size = MAX_SPEED * duration as f64 / ONE_HOUR;
    let contour = create_contour(start_coords, size, GRID_SIZE, &sptree, duration);
    println!(
        "Took {}ms to create contour",
        start_time.elapsed().as_millis()
    );

    match contour {
        Ok(c) => (Status::Ok, c),
        Err(_) => (
            Status::InternalServerError,
            "Error creating contour".to_string(),
        ),
    }
}

#[main]
async fn main() -> Result<(), rocket::Error> {
    let Args {
        osm_path,
        gtfs_path,
    } = Args::parse();

    // Build the graph
    let graph = build_graph_osm(&osm_path, &gtfs_path);

    // Start the server
    rocket::build()
        .configure(rocket::Config {
            workers: num_cpus::get(),
            ..rocket::Config::default()
        })
        .manage(graph)
        .mount("/isochrone", routes![isochrone])
        .launch()
        .await?;

    Ok(())
}
