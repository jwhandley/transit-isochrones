#[macro_use]
extern crate rocket;
use chrono::NaiveTime;
use clap::Parser;
use kdtree::KdTree;
use rocket::State;
use transit_isochrones::{
    alpha_shape, build_graph_osm, create_grid, dijkstra, geometry_to_geojson, Graph,
};

const PRECISION: f64 = 50.0; // Meters
const GRID_SIZE: f64 = 100.0; // Meters
const ONE_HOUR: f64 = 3600.0; // Seconds
const MAX_SPEED: f64 = 50_000.0; // Meters per second

#[derive(Parser, Debug)]
struct Args {
    osm_path: String,
    gtfs_path: String,
}

struct GraphState {
    graph: Graph,
    tree: KdTree<f64, i64, [f64; 2]>,
}

#[get("/?<lat>&<lon>&<arrival_time>&<duration>")]
fn isochrone(
    state: &State<GraphState>,
    lat: f64,
    lon: f64,
    arrival_time: &str,
    duration: u32,
) -> Result<String, String> {
    let start_coords = &[lon, lat];

    let arrival_time = match NaiveTime::parse_from_str(arrival_time, "%H:%M:%S") {
        Ok(time) => time,
        Err(_) => return Err("Invalid arrival time format. Please use HH:MM:SS.".to_string()),
    };

    let start_time = std::time::Instant::now();
    let sptree = match dijkstra(
        &state.graph,
        &state.tree,
        start_coords,
        arrival_time,
        duration,
    ) {
        Ok(tree) => tree,
        Err(_) => return Err("Coordinates not in graph or other processing error.".to_string()),
    };
    println!(
        "Took {}ms to find shortest path tree",
        start_time.elapsed().as_millis()
    );

    let size =  (duration as f64 / ONE_HOUR) * MAX_SPEED;
    let resolution = (size / GRID_SIZE) as usize;
    let grid = create_grid(start_coords, size, resolution, &sptree, duration);

    let geom = match alpha_shape(&grid, PRECISION) {
        Ok(geometry) => geometry,
        Err(_) => return Err("Error in computing alpha shape.".to_string()),
    };

    geometry_to_geojson(geom).map_err(|_| "Error converting geometry to GeoJSON.".to_string())
}

#[main]
async fn main() -> Result<(), rocket::Error> {
    let args = Args::parse();
    dbg!(&args);

    // Build the graph
    let (graph, tree) = build_graph_osm(&args.osm_path, &args.gtfs_path);
    let graph_state = GraphState { graph, tree };

    // Start the server
    rocket::build()
        .configure(rocket::Config {
            workers: num_cpus::get(), // sets the number of workers to the number of CPU cores
            ..rocket::Config::default()
        })
        .manage(graph_state)
        .mount("/isochrone", routes![isochrone])
        .launch()
        .await?;

    Ok(())
}
