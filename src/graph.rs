use gtfs_structures::Gtfs;
use kdtree::KdTree;
use osmpbf::{Element, ElementReader};
use std::collections::HashMap;

pub struct Edge {
    pub origin: i64,
    pub destination: i64,
    pub start_time: Option<u32>,
    pub end_time: Option<u32>,
    pub traversal_time: Option<u32>,
}

#[derive(PartialEq, Debug, Clone)]
pub struct Node {
    pub id: i64,
    pub x: f64,
    pub y: f64,
}

#[derive(Default)]
pub struct Graph {
    pub nodes: HashMap<i64, Node>,
    pub adjacency: HashMap<i64, Vec<Edge>>,
}

impl Graph {
    pub fn add_edge(
        &mut self,
        origin: i64,
        destination: i64,
        start_time: Option<u32>,
        end_time: Option<u32>,
        traversal_time: Option<u32>,
    ) {
        let edge = Edge {
            origin,
            destination,
            start_time,
            end_time,
            traversal_time,
        };
        self.adjacency.entry(origin).or_default().push(edge);
    }

    pub fn add_node(&mut self, index: i64, x: f64, y: f64) {
        let node = Node { id: index, x, y };
        self.nodes.insert(index, node);
    }

    pub fn neighbors(&self, index: i64) -> Option<&Vec<Edge>> {
        self.adjacency.get(&index)
    }
}

pub fn build_graph_osm(osm_path: &str, gtfs_path: &str) -> (Graph, KdTree<f64, i64, [f64; 2]>) {
    let mut osm_tree: KdTree<f64, i64, [f64; 2]> = KdTree::new(2);
    let mut graph = Graph::default();
    let mut max_index = 0;
    let reader = ElementReader::from_path(osm_path).unwrap();
    let gtfs = Gtfs::from_path(gtfs_path).unwrap();

    let start_time = std::time::Instant::now();
    println!("Adding OSM structure to graph");
    reader
        .for_each(|element| match element {
            Element::Node(osm_node) => {
                if is_walkable_node(&osm_node) {
                    let index = osm_node.id();
                    max_index = max_index.max(index);
                    let x = osm_node.lon();
                    let y = osm_node.lat();
                    graph.add_node(index, x, y);
                }
            }
            Element::Way(way) => {
                if is_walkable_way(&way) {
                    let mut nodes = way.refs();
                    let mut prev_node = nodes.next();
                    for curr_node in nodes {
                        max_index = max_index.max(curr_node);
                        if let Some(prev_id) = prev_node {
                            graph.add_edge(prev_id, curr_node, None, None, None);
                            graph.add_edge(curr_node, prev_id, None, None, None);
                            prev_node = Some(curr_node);
                        }
                    }
                }
            }
            Element::DenseNode(dense_node) => {
                max_index = max_index.max(dense_node.id());
                let x = dense_node.lon();
                let y = dense_node.lat();
                let id = dense_node.id();
                graph.add_node(id, x, y);
            }
            _ => {}
        })
        .unwrap();

    let mut nodes_without_edges = Vec::new();

    for (index, _) in graph.nodes.iter() {
        match graph.neighbors(*index) {
            Some(_) => {
                let id = *index;
                let lon = graph.nodes.get(index).unwrap().x;
                let lat = graph.nodes.get(index).unwrap().y;
                osm_tree.add([lon, lat], id).unwrap();
            }
            None => {
                nodes_without_edges.push(*index);
            }
        }
    }

    for i in nodes_without_edges.iter() {
        graph.nodes.remove(i);
        graph.adjacency.remove(i);
    }

    println!("Adding GTFS structure to graph");
    let mut stop_id_to_index: HashMap<&str, i64> = HashMap::new();
    for (stop_id, stop) in &gtfs.stops {
        let x = stop.longitude.unwrap();
        let y = stop.latitude.unwrap();

        let (distance, osm_node) = nearest_node(&osm_tree, &[x, y]).expect("No nearest node found");

        let index = *stop_id_to_index.entry(stop_id.as_str()).or_insert({
            max_index += 1;
            max_index
        });

        graph.add_node(index, x, y);

        let traversal_time = (distance / crate::dijkstra::WALKING_SPEED) as u32;
        graph.add_edge(index, osm_node, None, None, Some(traversal_time));
        graph.add_edge(osm_node, index, None, None, Some(traversal_time));

        let from_index = *stop_id_to_index.get(stop_id.as_str()).unwrap();
        for path in stop.pathways.iter() {
            let to_id = path.to_stop_id.as_str();

            let to_index = *stop_id_to_index.entry(to_id).or_insert({
                max_index += 1;

                max_index
            });

            match path.is_bidirectional {
                gtfs_structures::PathwayDirectionType::Unidirectional => {
                    graph.add_edge(from_index, to_index, None, None, path.traversal_time);
                }
                gtfs_structures::PathwayDirectionType::Bidirectional => {
                    graph.add_edge(from_index, to_index, None, None, path.traversal_time);
                    graph.add_edge(to_index, from_index, None, None, path.traversal_time);
                }
            }
        }
    }

    for (_, trip) in gtfs.trips.iter() {
        let mut stop_times = trip.stop_times.iter();
        let mut previous_stop = stop_times.next().unwrap();
        for stop_time in stop_times {
            let prev_index = *stop_id_to_index
                .get(previous_stop.stop.id.as_str())
                .unwrap();
            let curr_index = *stop_id_to_index.get(stop_time.stop.id.as_str()).unwrap();

            graph.add_edge(
                prev_index,
                curr_index,
                previous_stop.departure_time,
                stop_time.arrival_time,
                Some(stop_time.arrival_time.unwrap() - previous_stop.departure_time.unwrap()),
            );
            previous_stop = stop_time;
        }
    }

    for stop in gtfs.stops.values() {
        let id = stop_id_to_index.get(stop.id.as_str()).unwrap();
        let lon = stop.longitude.unwrap();
        let lat = stop.latitude.unwrap();
        osm_tree.add([lon, lat], *id).unwrap();
    }

    println!(
        "Took {}ms to build the graph",
        start_time.elapsed().as_millis()
    );
    (graph, osm_tree)
}

fn is_walkable_node(osm_node: &osmpbf::Node) -> bool {
    osm_node
        .tags()
        .any(|(key, value)| key == "foot" && value != "no")
}

fn is_walkable_way(way: &osmpbf::Way) -> bool {
    let mut is_highway_walkable = false;
    let mut foot_allowed = true;
    let mut service_allowed = true;

    for (key, value) in way.tags() {
        match key {
            "highway" => {
                is_highway_walkable = ![
                    "abandoned",
                    "bus_guideway",
                    "construction",
                    "cycleway",
                    "motor",
                    "no",
                    "planned",
                    "platform",
                    "proposed",
                    "raceway",
                    "razed",
                ]
                .contains(&value);
            }
            "foot" if value == "no" => {
                foot_allowed = false;
            }
            "service" if value == "private" => {
                service_allowed = false;
            }
            _ => {}
        }
    }

    is_highway_walkable && foot_allowed && service_allowed
}

fn nearest_node(tree: &KdTree<f64, i64, [f64; 2]>, coords: &[f64; 2]) -> Option<(f64, i64)> {
    let nearest = tree
        .nearest(coords, 1, &crate::dijkstra::haversine_distance)
        .expect("Should have been able to find node");

    if nearest.is_empty() {
        None
    } else {
        Some((nearest[0].0, *nearest[0].1))
    }
}
