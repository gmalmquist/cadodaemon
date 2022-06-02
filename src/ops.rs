use nalgebra::Point3;

pub enum Pivot {
    Origin,
    Centroid,
    Point(Point3<f64>),
}
