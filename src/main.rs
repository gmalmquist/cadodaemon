use crate::ops::Pivot;
use nalgebra::{Point3, Vector3};

mod mesh;
mod ops;

fn main() {
    println!("Hello, world!");

    let obj = obj::Obj::load(std::path::Path::new("./test-meshes/cube.obj"))
        .expect("Failed to load test input file ./test-meshes/cube.obj");
    let mut mesh: mesh::Mesh = obj.into();

    mesh.scale_mesh(Vector3::new(1., 2., 1.), Pivot::Origin);

    let obj: obj::Obj = mesh.into();

    obj.save(std::path::Path::new("./test-meshes/output.obj"))
        .expect("Failed to save output obj.");

    println!("Done.");
}
