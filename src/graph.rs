use gtfs_structures::Gtfs;
use kdtree::KdTree;
use osmpbf::{Element, ElementReader};
use std::{collections::HashMap, path::Path, sync::Arc};

#[derive(Eq, Hash, PartialEq, Clone)]
pub struct NodeId(Arc<str>);

pub struct TransportEdge {
    pub origin: NodeId,
    pub destination: NodeId,
    pub start_time: u32,
    pub end_time: u32,
}

pub struct WalkingEdge {
    pub origin: NodeId,
    pub destination: NodeId,
    pub traversal_time: Option<u32>,
}

pub enum Edge {
    Walking(WalkingEdge),
    Transport(TransportEdge),
}

impl Edge {
    pub fn dest(&self) -> NodeId {
        match &self {
            Edge::Walking(e) => e.destination.clone(),
            Edge::Transport(e) => e.destination.clone(),
        }
    }

    pub fn origin(&self) -> NodeId {
        match &self {
            Edge::Walking(e) => e.origin.clone(),
            Edge::Transport(e) => e.origin.clone(),
        }
    }
}

pub struct Node {
    pub lon: f64,
    pub lat: f64,
    edges: Vec<Edge>,
}

impl Default for Node {
    fn default() -> Self {
        Node {
            lon: 0.0,
            lat: 0.0,
            edges: Vec::new(),
        }
    }
}

pub struct Graph {
    pub nodes: HashMap<NodeId, Node>,
    pub tree: KdTree<f64, NodeId, [f64; 2]>,
}

impl Default for Graph {
    fn default() -> Self {
        Self {
            nodes: HashMap::new(),
            tree: KdTree::new(2),
        }
    }
}

impl Graph {
    pub fn add_edge(&mut self, edge: Edge) {
        let origin = edge.origin();
        self.nodes.entry(origin).or_default().edges.push(edge);
    }

    pub fn add_node(&mut self, index: NodeId, x: f64, y: f64) {
        let node = self.nodes.entry(index).or_default();

        node.lon = x;
        node.lat = y;
    }

    pub fn neighbors(&self, index: &NodeId) -> Option<&Vec<Edge>> {
        self.nodes.get(index).map(|node| &node.edges)
    }

    pub fn get_node(&self, index: &NodeId) -> Option<&Node> {
        self.nodes.get(index)
    }

    pub fn clear_edgeless_nodes(&mut self) {
        self.nodes.retain(|_, node| !node.edges.is_empty());
    }
}

pub fn build_graph_osm(osm_path: &Path, gtfs_path: &Path) -> Graph {
    let mut graph = Graph::default();

    let reader = ElementReader::from_path(osm_path).unwrap();
    let gtfs = Gtfs::from_path(
        gtfs_path
            .to_str()
            .expect("Should have been able to convert Path to str"),
    )
    .unwrap();

    let start_time = std::time::Instant::now();
    println!("Adding OSM structure to graph");
    reader
        .for_each(|element| match element {
            Element::Node(osm_node) => {
                if is_walkable_node(&osm_node) {
                    let index = NodeId(Arc::from(osm_node.id().to_string()));
                    let x = osm_node.lon();
                    let y = osm_node.lat();
                    graph.add_node(index, x, y);
                }
            }
            Element::Way(way) => {
                if is_walkable_way(&way) {
                    let refs: Vec<_> = way.refs().collect();

                    for window in refs.windows(2) {
                        let first = NodeId(Arc::from(window[0].to_string()));
                        let second = NodeId(Arc::from(window[1].to_string()));

                        graph.add_edge(Edge::Walking(WalkingEdge {
                            origin: first.clone(),
                            destination: second.clone(),
                            traversal_time: None,
                        }));
                        graph.add_edge(Edge::Walking(WalkingEdge {
                            origin: second,
                            destination: first,
                            traversal_time: None,
                        }));
                    }
                }
            }
            Element::DenseNode(dense_node) => {
                let x = dense_node.lon();
                let y = dense_node.lat();
                let id = NodeId(Arc::from(dense_node.id().to_string()));
                graph.add_node(id, x, y);
            }
            _ => {}
        })
        .unwrap();

    graph.clear_edgeless_nodes();

    for (id, node) in graph.nodes.iter() {
        let lon = node.lon;
        let lat = node.lat;
        graph.tree.add([lon, lat], id.clone()).unwrap();
    }

    println!("Adding GTFS structure to graph");
    for (stop_id, stop) in &gtfs.stops {
        let stop_id = NodeId(Arc::from(stop_id.clone()));
        let x = stop.longitude.unwrap();
        let y = stop.latitude.unwrap();

        let (distance, osm_node) =
            nearest_node(&graph.tree, &[x, y]).expect("No nearest node found");

        graph.add_node(stop_id.clone(), x, y);

        let traversal_time = (distance / crate::dijkstra::WALKING_SPEED) as u32;
        graph.add_edge(Edge::Walking(WalkingEdge {
            origin: stop_id.clone(),
            destination: osm_node.clone(),
            traversal_time: Some(traversal_time),
        }));
        graph.add_edge(Edge::Walking(WalkingEdge {
            origin: osm_node,
            destination: stop_id.clone(),
            traversal_time: Some(traversal_time),
        }));

        for path in stop.pathways.iter() {
            let to_id = NodeId(Arc::from(path.to_stop_id.clone()));

            match path.is_bidirectional {
                gtfs_structures::PathwayDirectionType::Unidirectional => {
                    graph.add_edge(Edge::Walking(WalkingEdge {
                        origin: stop_id.clone(),
                        destination: to_id,
                        traversal_time: path.traversal_time,
                    }));
                }
                gtfs_structures::PathwayDirectionType::Bidirectional => {
                    graph.add_edge(Edge::Walking(WalkingEdge {
                        origin: stop_id.clone(),
                        destination: to_id.clone(),
                        traversal_time: path.traversal_time,
                    }));
                    graph.add_edge(Edge::Walking(WalkingEdge {
                        origin: to_id.clone(),
                        destination: stop_id.clone(),
                        traversal_time: path.traversal_time,
                    }));
                }
            }
        }
    }

    for (_, trip) in gtfs.trips.iter() {
        for window in trip.stop_times.windows(2) {
            let origin = NodeId(Arc::from(window[0].stop.id.to_owned()));
            let destination = NodeId(Arc::from(window[1].stop.id.to_owned()));

            graph.add_edge(Edge::Transport(TransportEdge {
                origin,
                destination,
                start_time: window[0].departure_time.unwrap(),
                end_time: window[1].arrival_time.unwrap(),
            }));
        }
    }

    for stop in gtfs.stops.values() {
        let id = NodeId(Arc::from(stop.id.to_owned()));
        let lon = stop.longitude.unwrap();
        let lat = stop.latitude.unwrap();
        graph.tree.add([lon, lat], id).unwrap();
    }

    println!(
        "Took {}ms to build the graph",
        start_time.elapsed().as_millis()
    );

    graph
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

pub fn nearest_node<T>(
    tree: &KdTree<f64, T, [f64; 2]>,
    coords: &[f64; 2],
) -> Result<(f64, T), anyhow::Error>
where
    T: Eq + Clone,
{
    let nearest = tree.nearest(coords, 1, &crate::dijkstra::haversine_distance)?;
    Ok((nearest[0].0, nearest[0].1.clone()))
}
