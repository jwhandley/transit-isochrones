use crate::graph::nearest_node;
use crate::graph::Edge;
use crate::Graph;
use anyhow::anyhow;
use chrono::NaiveTime;
use chrono::Timelike;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::HashMap;

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
    position: String,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| self.position.cmp(&other.position))
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
) -> Result<HashMap<String, u32>, anyhow::Error> {
    let mut visited: HashMap<String, u32> = HashMap::new();
    let mut queue = BinaryHeap::new();
    let start_time = arrival_time.num_seconds_from_midnight() - duration;

    let (distance, start_node) =
        nearest_node(&graph.tree, start_coords).ok_or(anyhow!("No nearest node"))?;

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
        graph
            .neighbors(&position)
            .unwrap()
            .iter()
            .filter(|edge| match edge {
                Edge::Walking(_) => true,
                Edge::Transport(e) => e.start_time >= start_time + current_cost,
            })
            .for_each(|edge| {
                let new_cost = match edge {
                    Edge::Walking(e) => match e.traversal_time {
                        Some(time) => current_cost + time,
                        None => {
                            let start_node = graph.get_node(&edge.origin()).unwrap();
                            let end_node = graph.get_node(&edge.dest()).unwrap();
                            let distance = haversine_distance(
                                &[start_node.lon, start_node.lat],
                                &[end_node.lon, end_node.lat],
                            );

                            let walking_time = (distance / WALKING_SPEED) as u32;
                            current_cost + walking_time
                        }
                    },
                    Edge::Transport(e) => e.end_time - start_time,
                };

                let old_cost = visited.get(&edge.dest()).unwrap_or(&u32::MAX);
                if old_cost > &new_cost && new_cost <= duration {
                    visited.insert(edge.dest().clone(), new_cost);
                    let next_state = State {
                        cost: new_cost,
                        position: edge.dest(),
                    };
                    queue.push(next_state);
                }
            });
    }

    Ok(visited)
}
