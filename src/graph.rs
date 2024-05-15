use kdtree::KdTree;
use std::{collections::HashMap, sync::Arc};

#[derive(Eq, Hash, PartialEq, Clone)]
pub enum NodeId {
    Osm(i64),
    Gtfs(Arc<str>),
}

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
    pub fn dest(&self) -> &NodeId {
        match &self {
            Edge::Walking(e) => &e.destination,
            Edge::Transport(e) => &e.destination,
        }
    }

    pub fn origin(&self) -> &NodeId {
        match &self {
            Edge::Walking(e) => &e.origin,
            Edge::Transport(e) => &e.origin,
        }
    }
}

pub struct Node {
    pub lon: f64,
    pub lat: f64,
}

pub struct Graph {
    pub nodes: HashMap<NodeId, Node>,
    pub adjacency: HashMap<NodeId, Vec<Edge>>,
    pub tree: KdTree<f64, NodeId, [f64; 2]>,
}

impl Default for Graph {
    fn default() -> Self {
        Self {
            nodes: HashMap::new(),
            adjacency: HashMap::new(),
            tree: KdTree::new(2),
        }
    }
}

impl Graph {
    pub fn neighbors(&self, index: &NodeId) -> Option<&Vec<Edge>> {
        self.adjacency.get(index)
    }

    pub fn get_node(&self, index: &NodeId) -> Option<&Node> {
        self.nodes.get(index)
    }

    pub fn nearest_node(&self, coords: &[f64; 2]) -> Result<(f64, NodeId), anyhow::Error> {
        nearest_point(&self.tree, coords)
    }
}

pub fn nearest_point<T>(
    tree: &KdTree<f64, T, [f64; 2]>,
    coords: &[f64; 2],
) -> Result<(f64, T), anyhow::Error>
where
    T: Eq + Clone,
{
    let nearest = tree.nearest(coords, 1, &crate::dijkstra::haversine_distance)?;
    Ok((nearest[0].0, nearest[0].1.clone()))
}
