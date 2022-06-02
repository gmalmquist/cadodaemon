use crate::mesh::{EdgeId, Face, Group, Vertex};
use nalgebra::{Point3, Vector3};

pub enum Pivot {
    Origin,
    GlobalCentroid,
    IndividualCentroids,
    Point(Point3<f64>),
}

pub enum Constraints {
    None,
    OffAxis(Vector3<f64>),
    OnAxis(Vector3<f64>),
    OnNormals,
}

pub struct MeshSelection {
    vertices: Vec<Vertex>,
    edges: Vec<EdgeId>,
    faces: Vec<Face>,
    groups: Vec<Group>,
}
