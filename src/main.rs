#[macro_use] extern crate rocket;
use rocket::State;
use chrono::NaiveTime;
use clap::Parser;
use kdtree::KdTree;
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

#[get("/isochrone?<lat>&<lon>&<arrival_time>&<duration>")]
fn isochrone(state: &State<GraphState>, lat: f64, lon: f64, arrival_time: &str, duration: u32) -> String {
    let start_coords = &[lon, lat];
    let arrival_time = NaiveTime::parse_from_str(arrival_time, "%H:%M:%S").unwrap();
    let accessible_points = dijkstra(&state.graph, &state.tree, start_coords, arrival_time, duration);
    let geom = alpha_shape(&accessible_points, 75.0).unwrap();

    geometry_to_geojson(geom).unwrap()
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
        .manage(graph_state)
        .mount("/isochrone", routes![isochrone])
        .launch()
        .await?;

    Ok(())
}
