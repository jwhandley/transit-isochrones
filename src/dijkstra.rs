use crate::Graph;
use chrono::NaiveTime;
use chrono::Timelike;
use kdtree::KdTree;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

pub const WALKING_SPEED: f64 = 1.0;
const RADIUS_EARTH_KM: f64 = 6371.0;
const MAX_DISTANCE: f64 = 500.0;

pub fn haversine_distance(coord1: &[f64], coord2: &[f64]) -> f64 {
    let dlat = deg_to_rad(coord2[1] - coord1[1]);
    let dlon = deg_to_rad(coord2[0] - coord1[0]);

    let a = (dlat / 2.0).sin().powi(2)
        + coord1[1].to_radians().cos() * coord2[1].to_radians().cos() * (dlon / 2.0).sin().powi(2);

    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    RADIUS_EARTH_KM * c * 1000.0
}

fn deg_to_rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: u32,
    position: i64,
    coordinates: [f64; 2],
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

impl Eq for State {}

pub fn dijkstra(
    graph: &Graph,
    tree: &KdTree<f64, i64, [f64; 2]>,
    start_coords: &[f64; 2],
    arrival_time: NaiveTime,
    duration: u32,
) -> Result<KdTree<f64, u32, [f64; 2]>, anyhow::Error> {
    let mut visited = HashSet::new();
    let mut queue = BinaryHeap::new();
    let start_time = arrival_time.num_seconds_from_midnight() - duration;

    let nearest = tree
        .nearest(start_coords, 1, &haversine_distance)
        .expect("No nearest node");
    let start_node = *nearest[0].1;
    let distance = nearest[0].0;

    if distance > MAX_DISTANCE {
        Err(anyhow::anyhow!("Start node is too far away"))?;
    }

    let start_cost = start_time + (distance / WALKING_SPEED) as u32;

    queue.push(State {
        cost: start_cost,
        position: start_node,
        coordinates: *start_coords,
    });

    let mut reachable_coords = KdTree::new(2);

    while let Some(State {
        cost,
        position,
        coordinates,
    }) = queue.pop()
    {
        if visited.contains(&position) {
            continue;
        }
        visited.insert(position);

        let current_duration = cost - start_time;
        if current_duration <= duration {
            reachable_coords.add(coordinates, current_duration).unwrap();
        } else {
            continue;
        }

        for edge in graph
            .neighbors(position)
            .expect("There should be no nodes without edges")
        {
            // If there is a start time and it is before the current time, skip
            if let Some(start_time) = edge.start_time {
                if start_time < cost {
                    continue;
                }
            }

            let new_cost = edge.start_time.unwrap_or(cost);

            if let Some(end_time) = edge.end_time {
                if end_time > arrival_time.num_seconds_from_midnight() {
                    continue;
                }
            }

            let end_time = edge.end_time.unwrap_or(match edge.traversal_time {
                Some(t) => new_cost + t,
                None => {
                    let start_node = graph.nodes.get(&edge.origin).unwrap();
                    let end_node = graph.nodes.get(&edge.destination).unwrap();
                    let distance = haversine_distance(
                        &[start_node.x, start_node.y],
                        &[end_node.x, end_node.y],
                    );

                    let duration = (distance / WALKING_SPEED) as u32;
                    new_cost + duration
                }
            });

            if visited.contains(&edge.destination) {
                continue;
            }

            let next_state = State {
                cost: end_time,
                position: edge.destination,
                coordinates: [
                    graph.nodes.get(&edge.destination).unwrap().x,
                    graph.nodes.get(&edge.destination).unwrap().y,
                ],
            };

            queue.push(next_state);
        }
    }

    Ok(reachable_coords)
}
