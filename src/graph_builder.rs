use std::{collections::HashMap, path::Path, sync::Arc};

use anyhow::anyhow;
use gtfs_structures::Gtfs;
use kdtree::KdTree;
use osmpbf::{Element, ElementReader};

use crate::{
    graph::{nearest_point, Edge, Node, NodeId, TransportEdge, WalkingEdge},
    Graph,
};

pub struct GraphBuilder {
    nodes: HashMap<NodeId, Node>,
    adjacency: HashMap<NodeId, Vec<Edge>>,
    tree: KdTree<f64, NodeId, [f64; 2]>,
}

impl Default for GraphBuilder {
    fn default() -> Self {
        Self {
            nodes: HashMap::new(),
            adjacency: HashMap::new(),
            tree: KdTree::new(2),
        }
    }
}

impl GraphBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    fn add_edge(&mut self, edge: Edge) {
        let origin = edge.origin();
        self.adjacency.entry(origin.clone()).or_default().push(edge);
    }

    fn add_node(&mut self, index: NodeId, x: f64, y: f64) {
        let node = Node { lon: x, lat: y };
        self.nodes.insert(index, node);
    }

    pub fn load_osm(mut self, osm_path: &Path) -> Result<Self, anyhow::Error> {
        let reader = ElementReader::from_path(osm_path)?;
        reader.for_each(|element| match element {
            Element::Node(osm_node) => {
                if is_walkable_node(&osm_node) {
                    let id = NodeId::Osm(osm_node.id());
                    let lon = osm_node.lon();
                    let lat = osm_node.lat();
                    self.add_node(id, lon, lat);
                }
            }
            Element::Way(way) => {
                if is_walkable_way(&way) {
                    let refs: Vec<_> = way.refs().collect();

                    for window in refs.windows(2) {
                        let first = NodeId::Osm(window[0]);
                        let second = NodeId::Osm(window[1]);

                        self.add_edge(Edge::Walking(WalkingEdge {
                            origin: first.clone(),
                            destination: second.clone(),
                            traversal_time: None,
                        }));
                        self.add_edge(Edge::Walking(WalkingEdge {
                            origin: second,
                            destination: first,
                            traversal_time: None,
                        }));
                    }
                }
            }
            Element::DenseNode(dense_node) => {
                let id = NodeId::Osm(dense_node.id());
                let lon = dense_node.lon();
                let lat = dense_node.lat();

                self.add_node(id, lon, lat);
            }
            _ => {}
        })?;

        self.clear_edgeless_nodes();

        for (id, node) in self.nodes.iter() {
            let lon = node.lon;
            let lat = node.lat;
            self.tree.add([lon, lat], id.clone())?;
        }

        Ok(self)
    }

    pub fn load_gtfs(mut self, gtfs_path: &Path) -> Result<Self, anyhow::Error> {
        let gtfs = Gtfs::from_path(
            gtfs_path
                .to_str()
                .ok_or(anyhow!("Should have been able to convert path to str"))?,
        )?;

        for (stop_id, stop) in &gtfs.stops {
            let stop_id = NodeId::Gtfs(Arc::from(stop_id.clone()));
            let lon = stop
                .longitude
                .ok_or(anyhow!("All stops must have coordinates"))?;
            let lat = stop
                .latitude
                .ok_or(anyhow!("All stops must have coordinates"))?;

            let (distance, osm_node) = nearest_point(&self.tree, &[lon, lat])?;

            self.add_node(stop_id.clone(), lon, lat);

            let traversal_time = (distance / crate::dijkstra::WALKING_SPEED) as u32;
            self.add_edge(Edge::Walking(WalkingEdge {
                origin: stop_id.clone(),
                destination: osm_node.clone(),
                traversal_time: Some(traversal_time),
            }));
            self.add_edge(Edge::Walking(WalkingEdge {
                origin: osm_node,
                destination: stop_id.clone(),
                traversal_time: Some(traversal_time),
            }));

            for path in stop.pathways.iter() {
                let to_id = NodeId::Gtfs(Arc::from(path.to_stop_id.clone()));

                match path.is_bidirectional {
                    gtfs_structures::PathwayDirectionType::Unidirectional => {
                        self.add_edge(Edge::Walking(WalkingEdge {
                            origin: stop_id.clone(),
                            destination: to_id,
                            traversal_time: path.traversal_time,
                        }));
                    }
                    gtfs_structures::PathwayDirectionType::Bidirectional => {
                        self.add_edge(Edge::Walking(WalkingEdge {
                            origin: stop_id.clone(),
                            destination: to_id.clone(),
                            traversal_time: path.traversal_time,
                        }));
                        self.add_edge(Edge::Walking(WalkingEdge {
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
                let origin = NodeId::Gtfs(Arc::from(window[0].stop.id.to_owned()));
                let destination = NodeId::Gtfs(Arc::from(window[1].stop.id.to_owned()));

                self.add_edge(Edge::Transport(TransportEdge {
                    origin,
                    destination,
                    start_time: window[0]
                        .departure_time
                        .ok_or(anyhow!("All stops must have a departure time"))?,
                    end_time: window[1]
                        .arrival_time
                        .ok_or(anyhow!("All stops must have an arrival time"))?,
                }));
            }
        }

        for stop in gtfs.stops.values() {
            let id = NodeId::Gtfs(Arc::from(stop.id.to_owned()));
            let lon = stop
                .longitude
                .ok_or(anyhow!("All stops must have coordinates"))?;
            let lat = stop
                .latitude
                .ok_or(anyhow!("All stops must have coordinates"))?;
            self.tree.add([lon, lat], id)?;
        }

        Ok(self)
    }

    pub fn build(self) -> Graph {
        Graph {
            nodes: self.nodes,
            adjacency: self.adjacency,
            tree: self.tree,
        }
    }

    fn clear_edgeless_nodes(&mut self) {
        self.nodes.retain(|id, _| self.adjacency.contains_key(id));
    }
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
