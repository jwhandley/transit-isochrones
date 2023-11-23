#[macro_use]
extern crate rocket;
use chrono::NaiveTime;
use clap::Parser;
use kdtree::KdTree;
use rocket::State;
use transit_isochrones::{alpha_shape, build_graph_osm, dijkstra, geometry_to_geojson, Graph};

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

    let accessible_points = match dijkstra(
        &state.graph,
        &state.tree,
        start_coords,
        arrival_time,
        duration,
    ) {
        Ok(points) => points,
        Err(_) => return Err("Coordinates not in graph or other processing error.".to_string()),
    };

    let geom = match alpha_shape(&accessible_points, 75.0) {
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
            workers: num_cpus::get(),  // sets the number of workers to the number of CPU cores
            ..rocket::Config::default() })
        .manage(graph_state)
        .mount("/isochrone", routes![isochrone])
        .launch()
        .await?;

    Ok(())
}
