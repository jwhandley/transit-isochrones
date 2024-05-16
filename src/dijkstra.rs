use crate::graph::Edge;
use crate::graph::NodeId;
use crate::Graph;
use anyhow::anyhow;
use chrono::NaiveTime;
use chrono::Timelike;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::collections::HashSet;

pub const WALKING_SPEED: f64 = 1.0;
const RADIUS_EARTH_KM: f64 = 6371.0;
const MAX_DISTANCE: f64 = 500.0;

pub fn haversine_distance(coord1: &[f64], coord2: &[f64]) -> f64 {
    let dlat = (coord2[1] - coord1[1]).to_radians();
    let dlon = (coord2[0] - coord1[0]).to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + coord1[1].to_radians().cos() * coord2[1].to_radians().cos() * (dlon / 2.0).sin().powi(2);

    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    RADIUS_EARTH_KM * c * 1000.0
}

#[derive(Clone, PartialEq, Eq)]
struct State {
    cost: u32,
    position: NodeId,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn dijkstra(
    graph: &Graph,
    start_coords: &[f64; 2],
    arrival_time: NaiveTime,
    duration: u32,
) -> Result<HashMap<NodeId, u32>, anyhow::Error> {
    let mut distances: HashMap<NodeId, u32> = HashMap::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue = BinaryHeap::new();
    let start_time = arrival_time.num_seconds_from_midnight() - duration;

    let (distance, start_node) = graph.nearest_node(start_coords)?;

    if distance > MAX_DISTANCE {
        Err(anyhow!("Start node is too far away"))?;
    }

    let start_cost = (distance / WALKING_SPEED) as u32;

    queue.push(State {
        cost: start_cost,
        position: start_node,
    });

    while let Some(State {
        cost: current_cost,
        position,
    }) = queue.pop()
    {
        if visited.contains(&position) {
            continue;
        }

        let mut neighbors: Vec<_> = graph
            .neighbors(&position)
            .unwrap()
            .iter()
            .filter(|edge| {
                is_valid_edge(edge, start_time, current_cost) && !visited.contains(edge.dest())
            })
            .collect();

        neighbors.sort_unstable_by_key(|edge| match edge {
            Edge::Transport(e) => e.end_time,
            Edge::Walking(_) => u32::MIN,
        });

        neighbors.iter().for_each(|edge| {
            let new_cost = calculate_edge_cost(edge, current_cost, graph, start_time);
            let old_cost = distances.get(edge.dest()).unwrap_or(&u32::MAX);

            if old_cost > &new_cost && new_cost <= duration {
                distances.insert(edge.dest().clone(), new_cost);

                queue.push(State {
                    cost: new_cost,
                    position: edge.dest().clone(),
                });
            }
        });

        visited.insert(position);
    }

    Ok(distances)
}

fn is_valid_edge(edge: &Edge, start_time: u32, current_cost: u32) -> bool {
    match edge {
        Edge::Walking(_) => true,
        Edge::Transport(e) => e.start_time >= start_time + current_cost,
    }
}

fn calculate_edge_cost(edge: &Edge, current_cost: u32, graph: &Graph, start_time: u32) -> u32 {
    match edge {
        Edge::Walking(e) => match e.traversal_time {
            Some(time) => current_cost + time,
            None => current_cost + calculate_walking_time(graph, edge.origin(), edge.dest()),
        },
        Edge::Transport(e) => e.end_time - start_time,
    }
}

fn calculate_walking_time(graph: &Graph, start_node: &NodeId, end_node: &NodeId) -> u32 {
    let start_node = graph.get_node(start_node).unwrap();
    let end_node = graph.get_node(end_node).unwrap();
    let distance = haversine_distance(
        &[start_node.lon, start_node.lat],
        &[end_node.lon, end_node.lat],
    );
    (distance / WALKING_SPEED) as u32
}
